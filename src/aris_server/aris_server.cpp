#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


#include <cstring>
#include <thread>

#include "aris_core.h"
#include "aris_control.h"
#include "aris_server.h"

namespace aris
{
    namespace server
    {

        class ControlServer::Imp
        {
            public:
                auto loadXml(const aris::core::XmlDocument &doc)->void;
                auto addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const PlanFunc &gait_func)->void;
                auto start()->void;
                auto stop()->void;

                Imp(ControlServer *server)
                {
                    this->server_ = server;
                    this->controller_ = aris::control::EthercatController::createInstance<aris::control::EthercatController>();
                };
            private:
                enum RobotCmdID
                {
                    ENABLE,
                    DISABLE,
                    HOME,
                    RUN_GAIT,
                    FAKE_HOME,
                    ZERO_RUICONG,
                    JOG,

                    ROBOT_CMD_COUNT,
                    CLEAR_CMD_QUEUE
                };

            private:
                Imp(const Imp&) = delete;

                auto onReceiveMsg(const aris::core::Msg &msg)->aris::core::Msg;
                auto decodeMsg2Param(const aris::core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params)->void;
                auto sendParam(const std::string &cmd, const std::map<std::string, std::string> &params)->void;
                ParseFunc BasicCmdParseFunc(RobotCmdID cmd_id);
                ParseFunc JogCmdParseFunc();

                static auto tg(aris::control::EthercatController::Data &data)->int;
                auto run(GaitParamBase &param, aris::control::EthercatController::Data &data)->int;
                bool is_clear_cmd_queue_msg(char *cmd_param);
                int  discard_all_cmd(aris::control::EthercatController::Data& data);
                auto execute_cmd(int count, char *cmd, aris::control::EthercatController::Data &data)->int;
                auto enable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;
                auto disable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;
                auto home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;
                auto jog(const JogFunctionParam &param, aris::control::EthercatController::Data &data)->int;
                auto fake_home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;
                auto zero_ruicong(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;

            private:
                std::atomic_bool is_running_{false};

                ControlServer *server_;

                // 实时循环中的步态参数 //
                enum { CMD_POOL_SIZE = 50 };
                char cmd_queue_[CMD_POOL_SIZE][aris::core::MsgRT::RT_MSG_LENGTH];
                int current_cmd_{ 0 }, cmd_num_{ 0 }, count_{ 0 };

                // 以下储存所有的命令 //
                std::map<std::string, int> cmd_id_map_;//store gait id in follow vector
                std::vector<PlanFunc> plan_vec_;// store plan func
                std::vector<ParseFunc> parser_vec_; // store parse func
                std::map<std::string, std::unique_ptr<CommandStruct> > cmd_struct_map_;//store Node of command


                // 储存特殊命令的parse_func //
                ParseFunc parse_enable_func_     {BasicCmdParseFunc(RobotCmdID::ENABLE)};
                ParseFunc parse_disable_func_    {BasicCmdParseFunc(RobotCmdID::DISABLE)};
                ParseFunc parse_clear_queue_func_{BasicCmdParseFunc(RobotCmdID::CLEAR_CMD_QUEUE)};
                ParseFunc parse_home_func_       {BasicCmdParseFunc(RobotCmdID::HOME)};
                ParseFunc parse_fake_home_func_  {BasicCmdParseFunc(RobotCmdID::FAKE_HOME)};
                ParseFunc parse_zero_ruicong_    {BasicCmdParseFunc(RobotCmdID::ZERO_RUICONG)};
                ParseFunc parse_jog_func_        {JogCmdParseFunc()};


                // socket //
                aris::core::Socket server_socket_;
                std::string server_socket_ip_, server_socket_port_;

                // 储存控制器等 //
                aris::control::EthercatController *controller_;
                std::unique_ptr<aris::model::Model> model_;
//                std::unique_ptr<aris::sensor::IMU> imu_;

                // 结束时的callback //
                std::function<void(void)> on_exit_callback_{nullptr};

                std::vector<double> motion_pos_;

                // internal states for jog gait
                enum JOG_STATE
                {
                    NOT_READY = 0,
                    JOGGING   = 1,
                    STOPPING  = 2,
                    STOPPED   = 3
                };

                std::atomic_bool jog_stop_signal_received_{false};
                std::vector<JOG_STATE> motor_jog_states_;
                std::vector<std::size_t> jog_state_count_;
                std::vector<int> jog_stopping_vel_;
                const double JOG_DEFAULT_ACCEL = 160000;
                const int JOG_TIME_LIMIT_COUNT = 10000;
                
                friend class ControlServer;
        };

        auto ControlServer::Imp::loadXml(const aris::core::XmlDocument &doc)->void
        {
            /*load robot model_*/
            model_->loadXml(doc);

            /*begin to create imu*/
/*            if (doc.RootElement()->FirstChildElement("Sensors")->FirstChildElement("IMU")->Attribute("active", "true"))
            {
                std::cout << "imu found" << std::endl;
                imu_.reset(new aris::sensor::IMU(doc.RootElement()->FirstChildElement("Sensors")->FirstChildElement("IMU")));
            }
            else
            {
                std::cout << "imu not find" << std::endl;
            }
*/

            /*begin to load controller_*/
            controller_->loadXml(std::ref(*doc.RootElement()->
                        FirstChildElement("Controller")->
                        FirstChildElement("EtherCat")));

            controller_->setControlStrategy(tg);

            /*load connection param*/
            server_socket_ip_ = doc.RootElement()->FirstChildElement("Server")->Attribute("ip");
            server_socket_port_ = doc.RootElement()->FirstChildElement("Server")->Attribute("port");

            /*begin to insert cmd nodes*/
            auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");

            if (pCmds == nullptr) 
                throw std::runtime_error("invalid xml file, because it contains no commands information");
            cmd_struct_map_.clear();

            for (auto pChild = pCmds->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
            {
                if (cmd_struct_map_.find(pChild->name()) != cmd_struct_map_.end())
                    throw std::logic_error(std::string("command ") + pChild->name() + " is already existed, please rename it");

                cmd_struct_map_.insert(std::make_pair(std::string(pChild->name()), std::unique_ptr<CommandStruct>(new CommandStruct(pChild->name()))));
                AddAllParams(pChild, cmd_struct_map_.at(pChild->name())->root.get(), cmd_struct_map_.at(pChild->name())->allParams, cmd_struct_map_.at(pChild->name())->shortNames);
            }

            /*Set socket connection callback function*/
            server_socket_.setOnReceivedConnection([](aris::core::Socket *pConn, const char *pRemoteIP, int remotePort)
                    {
                    aris::core::log(std::string("received connection, the server_socket_ip_ is: ") + pRemoteIP);
                    return 0;
                    });
            server_socket_.setOnReceivedRequest([this](aris::core::Socket *pConn, aris::core::Msg &msg)
                    {
                    return onReceiveMsg(msg);
                    });
            server_socket_.setOnLoseConnection([this](aris::core::Socket *socket)
                    {
                        aris::core::log("lost connection");
                        while (true)
                        {
                            try
                            {
                                socket->startServer(this->server_socket_port_.c_str());
                                break;
                            }
                            catch (aris::core::Socket::StartServerError &e)
                            {
                                std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
                                aris::core::msSleep(1000);
                            }
                        }
                        aris::core::log("restart server socket successful");
                        return 0;
                    });
        }

        auto ControlServer::Imp::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const PlanFunc &gait_func)->void
        {
            if (cmd_name == "en")
            {
                if (gait_func)
                    throw std::runtime_error("you can not set plan_func for \"en\" command");
                this->parse_enable_func_ = parse_func;
            }
            else if (cmd_name == "ds")
            {
                if (gait_func)
                    throw std::runtime_error("you can not set plan_func for \"ds\" command");
                this->parse_disable_func_ = parse_func;
            }
            else if (cmd_name == "cq")
            {
                if (gait_func)
                    throw std::runtime_error("you can not set plan_func for \"cq\" command");
                this->parse_clear_queue_func_ = parse_func;
            }
            else if (cmd_name == "hm")
            {
                if (gait_func)
                    throw std::runtime_error("you can not set plan_func for \"hm\" command");
                this->parse_home_func_ = parse_func;
            }
            else if (cmd_name == "jog")
            {
                if (gait_func)
                    throw std::runtime_error("you can not set plan_func for \"jog\" command");
                this->parse_jog_func_ = parse_func;
            }
            else if (cmd_name == "fake_home")
            {
                if (gait_func)
                    throw std::runtime_error("you can not set plan_func for \"fake_home\" command");
                this->parse_home_func_ = parse_func;
            }
			else if (cmd_name == "zrc")
			{
				if (gait_func)
                    throw std::runtime_error("you can not set plan_func for \"zrc\" command");
				this->parse_zero_ruicong_ = parse_func;
			}
            else
            {
                if (cmd_id_map_.find(cmd_name) != cmd_id_map_.end())
                {
                    throw std::runtime_error(std::string("failed to add command, because \"") + cmd_name + "\" already exists");
                }
                else
                {
                    plan_vec_.push_back(gait_func);
                    parser_vec_.push_back(parse_func);

                    cmd_id_map_.insert(std::make_pair(cmd_name, plan_vec_.size() - 1));

                    std::cout << cmd_name << ":" << cmd_id_map_.at(cmd_name) << std::endl;
                }
            }
        };

        ParseFunc ControlServer::Imp::BasicCmdParseFunc(RobotCmdID cmd_id)
        {
            return [cmd_id, this](
                    const std::string &cmd, 
                    const std::map<std::string, std::string> &params,
                    aris::core::Msg &msg)
            {
                BasicFunctionParam param;
                param.cmd_type = cmd_id;
                std::fill_n(param.active_motor, this->controller_->motionNum(), true);
                msg.copyStruct(param);
            } ;
        }

        ParseFunc ControlServer::Imp::JogCmdParseFunc()
        {
            return [this](
                    const std::string &cmd, 
                    const std::map<std::string, std::string> &params,
                    aris::core::Msg &msg)
            {
                JogFunctionParam param;
                param.cmd_type = RobotCmdID::JOG;
                std::fill_n(param.active_motor, this->controller_->motionNum(), true);

                for (auto i : params)
                {
                    if (i.first == "vel")
                    {
                        param.jog_velocity_in_count = std::stoi(i.second);
                    }
                    else if (i.first == "acc")
                    {
                        param.jog_accel_in_count = std::stoi(i.second);
                    }
                    else if (i.first == "stop")
                    {
                        param.requireStop = (std::stoi(i.second)) == 1 ? true : false;
                    }
                }
                msg.copyStruct(param);
            };
        }

        auto ControlServer::Imp::start()->void
        {
            if (!is_running_)
            {
                is_running_ = true;
                motion_pos_.resize(controller_->motionNum());
                motor_jog_states_.resize(controller_->motionNum());
                jog_state_count_.resize(controller_->motionNum());
                jog_stopping_vel_.resize(controller_->motionNum());
//                if (imu_)imu_->start();
                controller_->start();
            }
        }
        auto ControlServer::Imp::stop()->void
        {
            if (is_running_)
            {
                controller_->stop();
//                if (imu_)imu_->stop();
                is_running_ = false;
            }
        }
        auto ControlServer::Imp::onReceiveMsg(const aris::core::Msg &msg)->aris::core::Msg
        {
            try
            {
                std::string cmd;
                std::map<std::string, std::string> params;

                decodeMsg2Param(msg, cmd, params);

                if (cmd == "start")
                {
                    if (is_running_)
                        throw std::runtime_error("server already started, thus ignore command \"start\"");
                    start();
                    return aris::core::Msg();
                }
                if (cmd == "stop")
                {
                    if (!is_running_)
                        throw std::runtime_error("server already stopped, thus ignore command \"stop\"");
                    stop();
                    return aris::core::Msg();
                }
                if (cmd == "exit")
                {
                    if (is_running_)
                        stop();

                    rt_printf("Server Imp stopped\n");

                    std::thread exit_callback([this]() 
                            {
                            aris::core::msSleep(1000);
                            if (on_exit_callback_)on_exit_callback_();
                            });

                    exit_callback.detach();

                    return aris::core::Msg();
                }

                if (!is_running_)throw std::runtime_error("can't execute command, because the server is not STARTED, please start it first");
                sendParam(cmd, params);

                return aris::core::Msg();
            }
            catch (std::exception &e)
            {
                std::cout << aris::core::log(e.what()) << std::endl;

                aris::core::Msg error_msg;
                error_msg.copy(e.what());
                return error_msg;
            }
            catch (...)
            {
                std::cout << aris::core::log("unknown exception") << std::endl;
                aris::core::Msg error_msg;
                error_msg.copy("unknown exception");
                return error_msg;
            }
        }
        auto ControlServer::Imp::decodeMsg2Param(const aris::core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params)->void
        {
            std::vector<std::string> paramVector;
            int paramNum{ 0 };

            /*将msg转换成cmd和一系列参数，不过这里的参数为原生字符串，既包括名称也包含值，例如“-heigt=0.5”*/
            if (msg.size() <= 0)
            {
                throw std::runtime_error(aris::core::log("Invalid message size"));
            }

            if (msg.data()[msg.size() - 1] == '\0')
            {
                std::string input{ msg.data() };
                std::stringstream inputStream{ input };
                std::string word;

                if (!(inputStream >> cmd))
                {
                    throw std::runtime_error(aris::core::log("invalid message from client, please at least contain a word"));
                };
                aris::core::log(std::string("received command string:") + msg.data());

                while (inputStream >> word)
                {
                    paramVector.push_back(word);
                    ++paramNum;
                }
            }
            else
            {
                throw std::runtime_error(aris::core::log("invalid message from client, please be sure that the command message end with char \'\\0\'"));
            }

            if (cmd_struct_map_.find(cmd) != cmd_struct_map_.end())
            {
                cmd_struct_map_.at(cmd)->root->Reset();
            }
            else
            {
                throw std::runtime_error(aris::core::log(std::string("invalid command name, server does not have command \"") + cmd + "\""));
            }

            for (int i = 0; i<paramNum; ++i)
            {
                std::string str{ paramVector[i] };
                std::string paramName, paramValue;
                if (str.find("=") == std::string::npos)
                {
                    paramName = str;
                    paramValue = "";
                }
                else
                {
                    paramName.assign(str, 0, str.find("="));
                    paramValue.assign(str, str.find("=") + 1, str.size() - str.find("="));
                }

                if (paramName.size() == 0)
                    throw std::runtime_error("invalid param: what the hell, param should not start with '='");

                /*not start with '-'*/
                if (paramName.data()[0] != '-')
                {
                    if (paramValue != "")
                    {
                        throw std::runtime_error("invalid param: only param start with - or -- can be assigned a value");
                    }

                    for (auto c : paramName)
                    {
                        if (cmd_struct_map_.at(cmd)->shortNames.find(c) != cmd_struct_map_.at(cmd)->shortNames.end())
                        {
                            params.insert(make_pair(cmd_struct_map_.at(cmd)->shortNames.at(c), paramValue));
                            cmd_struct_map_.at(cmd)->allParams.at(cmd_struct_map_.at(cmd)->shortNames.at(c))->Take();
                        }
                        else
                        {
                            throw std::runtime_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
                        }
                    }

                    continue;
                }

                /*all following part start with at least one '-'*/
                if (paramName.size() == 1)
                {
                    throw std::runtime_error("invalid param: symbol \"-\" must be followed by an abbreviation of param");
                }

                /*start with '-', but only one '-'*/
                if (paramName.data()[1] != '-')
                {
                    if (paramName.size() != 2)
                    {
                        throw std::runtime_error("invalid param: param start with single '-' must be an abbreviation");
                    }

                    char c = paramName.data()[1];

                    if (cmd_struct_map_.at(cmd)->shortNames.find(c) != cmd_struct_map_.at(cmd)->shortNames.end())
                    {
                        params.insert(make_pair(cmd_struct_map_.at(cmd)->shortNames.at(c), paramValue));
                        cmd_struct_map_.at(cmd)->allParams.at(cmd_struct_map_.at(cmd)->shortNames.at(c))->Take();
                    }
                    else
                    {
                        throw std::runtime_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
                    }

                    continue;
                }
                else
                {
                    /*start with '--'*/
                    if (paramName.size()<3)
                    {
                        throw std::runtime_error("invalid param: symbol \"--\" must be followed by a full name of param");
                    }

                    std::string str = paramName;
                    paramName.assign(str, 2, str.size() - 2);

                    if (cmd_struct_map_.at(cmd)->allParams.find(paramName) != cmd_struct_map_.at(cmd)->allParams.end())
                    {
                        params.insert(make_pair(paramName, paramValue));
                        cmd_struct_map_.at(cmd)->allParams.at(paramName)->Take();
                    }
                    else
                    {
                        throw std::runtime_error(std::string("invalid param: param \"") + paramName + "\" is not a valid param");
                    }



                    continue;
                }
            }

            AddAllDefault(cmd_struct_map_.at(cmd)->root.get(), params);

            std::cout << cmd << std::endl;

            int paramPrintLength;
            if (params.empty())
            {
                paramPrintLength = 2;
            }
            else
            {
                paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
                        {
                        return a.first.length() < b.first.length();
                        })->first.length() + 2;
            }

            int maxParamNameLength{ 0 };
            for (auto &i : params)
            {
                std::cout << std::string(paramPrintLength - i.first.length(), ' ') << i.first << " : " << i.second << std::endl;
            }
        }

        auto ControlServer::Imp::sendParam(const std::string &cmd, const std::map<std::string, std::string> &params)->void
        {
            aris::core::Msg cmd_msg;

            if (cmd == "en")
            {
                parse_enable_func_(cmd, params, cmd_msg);
                if (cmd_msg.size() != sizeof(BasicFunctionParam))
                    throw std::runtime_error("invalid msg length of parse function for en");
                reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::ENABLE;
            }
            else if (cmd == "ds")
            {
                parse_disable_func_(cmd, params, cmd_msg);
                if (cmd_msg.size() != sizeof(BasicFunctionParam))
                    throw std::runtime_error("invalid msg length of parse function for ds");
                reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::DISABLE;
            }
            else if (cmd == "cq")
            {
                std::cout << "Clear Queue message is received, all commands are canceled and all motors are disabled" << std::endl;
                parse_clear_queue_func_(cmd, params, cmd_msg);
                if (cmd_msg.size() != sizeof(BasicFunctionParam))
                    throw std::runtime_error("invalid msg length of parse function for cq");

                reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::CLEAR_CMD_QUEUE;
            }
            else if (cmd == "hm")
            {
                parse_home_func_(cmd, params, cmd_msg);
                if (cmd_msg.size() != sizeof(BasicFunctionParam))
                    throw std::runtime_error("invalid msg length of parse function for hm");
                reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::HOME;
            }
            else if (cmd == "jog")
            {
                parse_jog_func_(cmd, params, cmd_msg);
                if (cmd_msg.size() != sizeof(JogFunctionParam))
                    throw std::runtime_error("invalid msg length of parse function for jog");
                JogFunctionParam* paramPtr = reinterpret_cast<JogFunctionParam *>(cmd_msg.data());
                paramPtr->cmd_type = ControlServer::Imp::JOG;
                
                // post process to change internal jog state
                if (paramPtr->requireStop)
                {
                    jog_stop_signal_received_ = true;
                    // not queue this message since we only want to change the stop bit for ongoing jogging cmd
                    cmd_msg.setNotQueueFlag(true);
                }
                else
                {
                    jog_stop_signal_received_ = false;
                }
            }
            else if (cmd == "fake_home")
            {
                parse_fake_home_func_(cmd, params, cmd_msg);
                if (cmd_msg.size() != sizeof(BasicFunctionParam))
                    throw std::runtime_error("invalid msg length of parse function for fake_home");
                reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::FAKE_HOME;
            }
			else if (cmd == "zrc")
			{
				parse_zero_ruicong_(cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))
				    throw std::runtime_error("invalid msg length of parse function for fake_home");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::ZERO_RUICONG;
			}
            else
            {
                auto cmdPair = this->cmd_id_map_.find(cmd);

                if (cmdPair == this->cmd_id_map_.end())
                {
                    throw std::runtime_error(std::string("command \"") + cmd + "\" does not have gait function, please AddCmd() first");
                }

                this->parser_vec_.at(cmdPair->second).operator()(cmd, params, cmd_msg);

                if (cmd_msg.size() < sizeof(GaitParamBase))
                {
                    throw std::runtime_error(std::string("parse function of command \"") + cmdPair->first + "\" failed: because it returned invalid cmd_msg");
                }

                reinterpret_cast<GaitParamBase *>(cmd_msg.data())->cmd_type = RUN_GAIT;
                reinterpret_cast<GaitParamBase *>(cmd_msg.data())->gait_id = cmdPair->second;

                if (plan_vec_.at(cmdPair->second) == nullptr) return;
            }

            cmd_msg.setMsgID(0);

            
            if (!cmd_msg.notQueueFlag())
                controller_->msgPipe().sendToRT(cmd_msg);
        }

        auto ControlServer::Imp::tg(aris::control::EthercatController::Data &data)->int
        {
            static ControlServer::Imp *imp = ControlServer::instance().imp.get();

            // 检查是否出错 //
            static int fault_count = 0;

            auto error_motor = std::find_if(
                data.motion_raw_data->begin(), 
                data.motion_raw_data->end(), 
                [](const aris::control::EthercatMotion::RawData &data) 
                {
                    return data.ret < 0; 
                });

            if (error_motor != data.motion_raw_data->end())
            {
                fault_count++;
                if (fault_count % 100 == 0)
                {
                    for (auto &mot_data : *data.motion_raw_data)rt_printf("%d ", mot_data.ret);

                    rt_printf("\n");

                    rt_printf("Some motor is in fault, all motors would be disabled if \
                               such fault lasts for 1 second\n");
                }
                
                if (fault_count % 1000 == 0)
                {
                    imp->discard_all_cmd(data);
                    rt_printf("All commands in command queue are discarded\n");
                    rt_printf("All motors are to be disabled now\n");
                    return 0;
                }
            }
            else
            {
                fault_count = 0;
            }

            // 查看是否有新cmd //
            if (data.msg_recv)
            {
                if (imp->cmd_num_ >= CMD_POOL_SIZE)
                {
                    rt_printf("cmd pool is full, thus ignore last one\n");
                }
                else
                {
                    data.msg_recv->paste(imp->cmd_queue_[(imp->current_cmd_ + imp->cmd_num_) % CMD_POOL_SIZE]);
                    char *recv_cmd = imp->cmd_queue_[(imp->current_cmd_ + imp->cmd_num_) % CMD_POOL_SIZE];
                    ++imp->cmd_num_;
                    
                    if (imp->is_clear_cmd_queue_msg(recv_cmd))
                    {
                        // discard all commands, including the executing command
                        rt_printf("Clear queue cmd received, discard all commands\n");
                        imp->discard_all_cmd(data);
                        return 0;
                    }
                }
            }

            // 执行cmd queue中的cmd //
            if (imp->cmd_num_ > 0)
            {
                auto ret = imp->execute_cmd(imp->count_, imp->cmd_queue_[imp->current_cmd_], data);
                if (ret == 0)
                {
                    rt_printf("cmd finished, spend %d counts\n\n", imp->count_ + 1);
                    imp->count_ = 0;
                    imp->current_cmd_ = (imp->current_cmd_ + 1) % CMD_POOL_SIZE;
                    --imp->cmd_num_;
                }
                else
                {
                    imp->count_++;
                    if (imp->count_ % 1000 == 0)
                        rt_printf("execute cmd in count: %d\n", imp->count_);
                }
            }

            return 0;
        }

        int ControlServer::Imp::discard_all_cmd(aris::control::EthercatController::Data& data)
        {
            for (auto &mot_data : *data.motion_raw_data)
            {
                mot_data.cmd = aris::control::EthercatMotion::DISABLE;
            }

            cmd_num_ = 0;
            count_ = 0;
            return 0;
        }

        bool ControlServer::Imp::is_clear_cmd_queue_msg(char *cmd_param)
        {
            PlanParamBase *param = reinterpret_cast<PlanParamBase *>(cmd_param);
            if (param->cmd_type == CLEAR_CMD_QUEUE)
                return true;

            return false;
        }

        auto ControlServer::Imp::execute_cmd(int count, char *cmd_param, aris::control::EthercatController::Data &data)->int
        {
            int ret;
            PlanParamBase *param = reinterpret_cast<PlanParamBase *>(cmd_param);
            param->count = count;

            switch (param->cmd_type)
            {
                case ENABLE:
                    ret = enable(static_cast<BasicFunctionParam &>(*param), data);
                    break;
                case DISABLE:
                    ret = disable(static_cast<BasicFunctionParam &>(*param), data);
                    break;
                case HOME:
                    ret = home(static_cast<BasicFunctionParam &>(*param), data);
                    break;
                case FAKE_HOME:
                    ret = fake_home(static_cast<BasicFunctionParam &>(*param), data);
                    break;
                case RUN_GAIT:
                    ret = run(static_cast<GaitParamBase &>(*param), data);
                    break;
                case ZERO_RUICONG:
                    ret = zero_ruicong(static_cast<BasicFunctionParam &>(*param), data);
                    break;
                case JOG:
                    ret = jog(static_cast<JogFunctionParam &>(*param), data);
                    break;
                default:
                    rt_printf("unknown cmd type\n");
                    ret = 0;
                    break;
            }

            return ret;
        }
        auto ControlServer::Imp::enable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
        {
            bool is_all_enabled = true;

            for (std::size_t i = 0; i < controller_->motionNum(); ++i)
            {
                if (param.active_motor[i])
                {
                    /*判断是否已经Enable了*/
                    if ((param.count != 0) && (data.motion_raw_data->operator[](i).ret == 0))
                    {
                        /*判断是否为第一次走到enable,否则什么也不做，这样就会继续刷上次的值*/
                        if (data.motion_raw_data->operator[](i).cmd == aris::control::EthercatMotion::ENABLE)
                        {
                            data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).mode = aris::control::EthercatMotion::POSITION;
                            data.motion_raw_data->operator[](i).target_pos = data.motion_raw_data->operator[](i).feedback_pos;
                            data.motion_raw_data->operator[](i).target_vel = 0;
                            data.motion_raw_data->operator[](i).target_cur = 0;
                        }
                    }
                    else
                    {
                        is_all_enabled = false;
                        data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::ENABLE;
                        data.motion_raw_data->operator[](i).mode = aris::control::EthercatMotion::POSITION;

                        if (param.count % 1000 == 0)
                        {
                            rt_printf("Unenabled motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
                        }
                    }
                }
            }

            return is_all_enabled ? 0 : 1;
        };
        auto ControlServer::Imp::disable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
        {
            bool is_all_disabled = true;

            for (std::size_t i = 0; i < controller_->motionNum(); ++i)
            {
                if (param.active_motor[i])
                {
                    /*判断是否已经Disabled了*/
                    if ((param.count != 0) && (data.motion_raw_data->operator[](i).ret == 0))
                    {
                        /*如果已经disable了，那么什么都不做*/
                    }
                    else
                    {
                        /*否则往下刷disable指令*/
                        is_all_disabled = false;
                        data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::DISABLE;

                        if (param.count % 1000 == 0)
                        {
                            rt_printf("Undisabled motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
                        }
                    }
                }
            }

            return is_all_disabled ? 0 : 1;
        }
        auto ControlServer::Imp::home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
        {
            bool is_all_homed = true;

            for (std::size_t i = 0; i < controller_->motionNum(); ++i)
            {
                if (param.active_motor[i])
                {
                    // 将电机的偏置置为0 //
                    controller_->motionAtAbs(i).setPosOffset(0);

                    // 根据返回值来判断是否走到home了 //
                    if ((param.count != 0) && (data.motion_raw_data->operator[](i).ret == 0))
                    {
                        // 判断是否为第一次走到home,否则什么也不做，这样就会继续刷上次的值 //
                        if (data.motion_raw_data->operator[](i).cmd == aris::control::EthercatMotion::HOME)
                        {
                            data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).target_pos = data.motion_raw_data->operator[](i).feedback_pos;
                            data.motion_raw_data->operator[](i).target_vel = 0;
                            data.motion_raw_data->operator[](i).target_cur = 0;
                        }
                    }
                    else
                    {
                        is_all_homed = false;
                        data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::HOME;

                        if (param.count % 1000 == 0)
                        {
                            rt_printf("Unhomed motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
                        }
                    }
                }
            }

            return is_all_homed ? 0 : 1;
        };

        auto ControlServer::Imp::jog(const JogFunctionParam &param, aris::control::EthercatController::Data &data)->int
        {
            using aris::control::EthercatMotion;

            bool is_all_jog_finished = true;

            for (std::size_t i = 0; i < controller_->motionNum(); ++i)
            {
                if (param.active_motor[i])
                {
                    if (param.count == 0)
                    {
                        // initalize jog states for each motor
                        motor_jog_states_[i] = JOG_STATE::NOT_READY;
                        jog_state_count_[i] = param.count+1;
                        jog_stop_signal_received_ = false;
                        is_all_jog_finished = false;
                    }
                    else if (motor_jog_states_[i] == JOG_STATE::NOT_READY)
                    {
                        // check if this motor have been enabled with desired mode
                        if (param.count != jog_state_count_[i] && data.motion_raw_data->operator[](i).ret == 0)
                        {
                            motor_jog_states_[i] = JOG_STATE::JOGGING;
                            jog_state_count_[i] = param.count+1;
                        }
                        else{
                            // enable this motor and switch it to VELOCITY mode
                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::ENABLE;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::VELOCITY;
                            data.motion_raw_data->operator[](i).target_vel = 0;
                        }
                        is_all_jog_finished = false;
                    }
                    else if (motor_jog_states_[i] == JOG_STATE::JOGGING)
                    {
                        // check if the stop signal is received or the time limit is reached
                        bool time_limit_reached = (param.count > (jog_state_count_[i] + JOG_TIME_LIMIT_COUNT));
                        if (jog_stop_signal_received_ || time_limit_reached)
                        {
                            motor_jog_states_[i] = JOG_STATE::STOPPING;
                            jog_state_count_[i] = param.count+1;
                            jog_stopping_vel_[i] = data.motion_raw_data->operator[](i).feedback_vel;
                        }
                        else
                        {
                            int desired_vel = param.jog_velocity_in_count;
                            double jog_acc = param.jog_accel_in_count;

                            // avoid singularity
                            if (fabs(jog_acc) < 1)
                                jog_acc = JOG_DEFAULT_ACCEL;

                            // make jog_acc the same sign with the vel
                            if (desired_vel < 0)
                                jog_acc = -fabs(jog_acc);

                            // jog motor 
                            int jogging_count = param.count - jog_state_count_[i];
                            if (jogging_count < fabs(desired_vel / jog_acc)*1000)
                            {
                                // accel motor
                                data.motion_raw_data->operator[](i).target_vel = (int)(jog_acc/1000 * jogging_count);
                            }
                            else
                            {
                                // constant speed jogging
                                data.motion_raw_data->operator[](i).target_vel = desired_vel;
                            }

                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::VELOCITY;
                        }
                        is_all_jog_finished = false;
                    }
                    else if (motor_jog_states_[i] == JOG_STATE::STOPPING)
                    {
                        double jog_dec = param.jog_accel_in_count;
                        if (fabs(jog_dec) < 1) 
                            jog_dec = JOG_DEFAULT_ACCEL;

                        // make jog_acc the same sign with the vel
                        if (jog_stopping_vel_[i] < 0)
                            jog_dec = -fabs(jog_dec);

                        int deccel_count = fabs(jog_stopping_vel_[i] / jog_dec)*1000;
                        //check if the motor comes to a stop
                        if (param.count > (jog_state_count_[i]+deccel_count + 1))
                        {
                            motor_jog_states_[i] = JOG_STATE::STOPPED;
                            jog_state_count_[i] = param.count+1;
                        }
                        else
                        {
                            int stopping_count = param.count - jog_state_count_[i];
                            // deccel motor 
                            if (stopping_count < deccel_count)
                            {
                                data.motion_raw_data->operator[](i).target_vel = 
                                    (int)(jog_stopping_vel_[i] - jog_dec/1000 * stopping_count);
                            }
                            else
                            {
                                data.motion_raw_data->operator[](i).target_vel = 0;
                            }
                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::VELOCITY;
                        }
                        
                        is_all_jog_finished = false;
                    }
                    else // the motor has reached the JOG_STATE::STOPPED state
                    {
                        if (param.count == jog_state_count_[i])
                        {
                            // switch motor back to the POSITION mode
                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::POSITION;
                            data.motion_raw_data->operator[](i).target_pos = data.motion_raw_data->operator[](i).feedback_pos;
                            data.motion_raw_data->operator[](i).target_vel = 0;
                            data.motion_raw_data->operator[](i).target_cur = 0;
                            rt_printf("Motor %d jog stopped\n", i);
                        }
                    }
                }
            }

            if (is_all_jog_finished)
                jog_stop_signal_received_ = false;

            return is_all_jog_finished ? 0 : 1;
        }

        auto ControlServer::Imp::fake_home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
        {
            for (std::size_t i = 0; i < controller_->motionNum(); ++i)
            {
                auto& motion = controller_->motionAtAbs(i);
                motion.setPosOffset( static_cast<std::int32_t>(
                            motion.posOffset() + motion.homeCount() - data.motion_raw_data->at(i).feedback_pos
                            ));
            }

            for (std::size_t i = 0; i < controller_->motionNum(); ++i){
                rt_printf("Pos2CountRatio %d\n", controller_->motionAtAbs(i).pos2countRatio());
                rt_printf("Motpos: %.3f FeedbackPos: %d  posOffset: %d\n",
                        controller_->motionAtAbs(i).homeCount(),
                        data.motion_raw_data->at(i).feedback_pos,
                        controller_->motionAtAbs(i).posOffset()
                        );
            }

            return 0;
        };
        
		auto ControlServer::Imp::zero_ruicong(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
		{
			for (std::size_t i = 0; i < data.ruicongcombo_data->at(0).isZeroingRequested.size(); i++)
			{
				if (param.active_channel[i])
				{
					data.ruicongcombo_data->at(0).isZeroingRequested.at(i) = true;
					rt_printf("zeroing channel: %d\n",i);
				}
				else
				{
					data.ruicongcombo_data->at(0).isZeroingRequested.at(i) = false;
					rt_printf("not zeroing channel: %d\n", i);
				}
			}
			return 0;
		}

        auto ControlServer::Imp::run(GaitParamBase &param, aris::control::EthercatController::Data &data)->int
        {
            static ControlServer::Imp *imp = ControlServer::instance().imp.get();

            // 获取陀螺仪传感器数据 //
/*            aris::sensor::SensorData<aris::sensor::ImuData> imuDataProtected;
            if (imu_) imuDataProtected = imu_->getSensorData();
            param.imu_data = &imuDataProtected.get();
*/
            // 获取力传感器数据与电机数据 //
            param.force_data = data.force_sensor_data;
            param.motion_raw_data = data.motion_raw_data;
            param.ruicong_data = data.ruicongcombo_data;
            param.imu_data = data.imu_data;
            param.last_motion_raw_data = data.last_motion_raw_data;
            param.motion_feedback_pos = &this->motion_pos_;

            for (std::size_t i = 0; i < data.motion_raw_data->size(); ++i)
            {
                this->motion_pos_[i] = 
                    static_cast<double>(data.motion_raw_data->at(i).feedback_pos) /
                    controller_->motionAtAbs(i).pos2countRatio();
            }

            // 执行gait函数 //
            PlanFunc func = this->plan_vec_.at(param.gait_id);
            int ret = func(*model_.get(), param);

            // write cmds to drives //
            for (std::size_t i = 0; i < controller_->motionNum(); ++i)
            {
                if (param.active_motor[i])
                {
                    auto driveData = data.motion_raw_data->operator[](i); 
                    auto controlData = param.motion_raw_data->operator[](i); 
                    
                    // only write outputs, since the PlanFunc could modify feedback data
                    driveData.cmd = controlData.cmd;
                    driveData.mode = controlData.mode;
                    driveData.target_pos = controlData.target_pos;
                    driveData.target_vel = controlData.target_vel;
                    driveData.target_cur = controlData.target_cur;
                }
            }

            // 检查位置极限和速度是否连续 //
            for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
            {
                if (data.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN)
                {
                    if (param.if_check_pos_max && 
                        (data.motion_raw_data->at(i).target_pos > imp->controller_->motionAtAbs(i).maxPosCount()))
                    {
                        rt_printf("Motor %i's target position is bigger than its MAX permitted value in count:%d\n", i, imp->count_);
                        rt_printf("The min, max and current count are:\n");
                        for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
                        {
                            rt_printf("%d   %d   %d\n", 
                                    imp->controller_->motionAtAbs(i).minPosCount(), 
                                    imp->controller_->motionAtAbs(i).maxPosCount(), 
                                    data.motion_raw_data->at(i).target_pos);
                        }
                        rt_printf("All commands in command queue are discarded, please try to RECOVER\n");
                        imp->cmd_num_ = 1;//因为这里为0退出，因此之后在tg中回递减cmd_num_,所以这里必须为1
                        imp->count_ = 0;

                        // 发现不连续，那么使用上一个成功的cmd，以便等待修复 //
                        for (std::size_t i = 0; i < imp->controller_->motionNum(); ++i)
                            data.motion_raw_data->operator[](i) = data.last_motion_raw_data->operator[](i);

                        return 0;
                    }

                    if (param.if_check_pos_min && 
                        (data.motion_raw_data->at(i).target_pos < imp->controller_->motionAtAbs(i).minPosCount()))
                    {
                        rt_printf("Motor %i's target position is smaller than its MIN permitted value in count:%d\n", i, imp->count_);
                        rt_printf("The min, max and current count are:\n");
                        for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
                        {
                            rt_printf("%d   %d   %d\n", 
                                    imp->controller_->motionAtAbs(i).minPosCount(), 
                                    imp->controller_->motionAtAbs(i).maxPosCount(), 
                                    data.motion_raw_data->at(i).target_pos);
                        }
                        rt_printf("All commands in command queue are discarded, please try to RECOVER\n");
                        imp->cmd_num_ = 1;//因为这里为0退出，因此之后在tg中回递减cmd_num_,所以这里必须为1
                        imp->count_ = 0;

                        // 发现不连续，那么使用上一个成功的cmd，以便等待修复 //
                        for (std::size_t i = 0; i < imp->controller_->motionNum(); ++i)
                            data.motion_raw_data->operator[](i) = data.last_motion_raw_data->operator[](i);

                        return 0;
                    }

                    if (param.if_check_pos_continuous && 
                        (std::abs(data.last_motion_raw_data->at(i).target_pos - data.motion_raw_data->at(i).target_pos)
                            > 0.0012*imp->controller_->motionAtAbs(i).maxVelCount())
                       )
                    {
                        rt_printf("Motor %i's target position is not continuous in count:%d\n", i, imp->count_);

                        rt_printf("The input of last and this count are:\n");
                        for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
                        {
                            rt_printf("%d   %d\n", 
                                    data.last_motion_raw_data->at(i).target_pos, 
                                    data.motion_raw_data->at(i).target_pos);
                        }

                        rt_printf("All commands in command queue are discarded, please try to RECOVER\n");
                        imp->cmd_num_ = 1;//因为这里为0退出，因此之后在tg中回递减cmd_num_,所以这里必须为1
                        imp->count_ = 0;

                        // 发现不连续，那么使用上一个成功的cmd，以便等待修复 //
                        for (std::size_t i = 0; i < imp->controller_->motionNum(); ++i)
                            data.motion_raw_data->operator[](i) = data.last_motion_raw_data->operator[](i);

                        return 0;
                    }
                }
            }

            return ret;
        }

        ControlServer &ControlServer::instance()
        {
            static ControlServer instance;
            return std::ref(instance);
        }

        ControlServer::ControlServer() :imp(new Imp(this)) {}

        ControlServer::~ControlServer() {}

        auto ControlServer::createModel(model::Model *model_)->void
        {
            if (imp->model_)
                throw std::runtime_error("control sever can't create model because it already has one");

            imp->model_.reset(model_);
        }

        auto ControlServer::loadXml(const char *fileName)->void
        {
            aris::core::XmlDocument doc;

            if (doc.LoadFile(fileName) != 0)
            {
                throw std::logic_error((std::string("could not open file:") + std::string(fileName)));
            }

            loadXml(doc);
        }

        auto ControlServer::loadXml(const aris::core::XmlDocument &xmlDoc)->void 
        {
            imp->loadXml(xmlDoc);
        }

        auto ControlServer::model()->model::Model&
        {
            return std::ref(*imp->model_.get());
        };

        auto ControlServer::controller()->control::EthercatController&
        {
            return std::ref(*imp->controller_);
        }

        auto ControlServer::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const PlanFunc &gait_func)->void
        {
            imp->addCmd(cmd_name, parse_func, gait_func);
        }

        auto ControlServer::open()->void 
        {
            for (;;)
            {
                try
                {
                    imp->server_socket_.startServer(imp->server_socket_port_.c_str());
                    break;
                }
                catch (aris::core::Socket::StartServerError &e)
                {
                    std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
                    aris::core::msSleep(1000);
                }
            }
            std::cout << aris::core::log("server open successful") << std::endl;
        };
        auto ControlServer::close()->void 
        {
            imp->server_socket_.stop();
        };
        auto ControlServer::setOnExit(std::function<void(void)> callback_func)->void
        {
            this->imp->on_exit_callback_ = callback_func;
        }
    }
}

