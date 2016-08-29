#ifdef UNIX
#include <ecrt.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <sys/mman.h>
#endif
#ifdef WIN32
#define rt_printf printf
#endif



#include <string>
#include <iostream>
#include <map>
#include <fstream>
#include <algorithm>

#include "aris_control_motion.h"


namespace aris
{
	namespace control
	{
#ifdef UNIX
    namespace data_emitter
    {

    auto Data_Emitter::setUDP(std::string d_addr,std::string d_port,std::string l_port)->void
    {
        this->dest_addr=d_addr;
        this->dest_port=d_port;
        this->local_port=l_port;
        std::cout<<this->dest_addr<<std::endl<<this->dest_port<<std::endl<<this->local_port<<std::endl;

    };

    auto Data_Emitter::dataEmitterPipe()->aris::control::Pipe<Data>&
    {
        return this->data_emitter_pipe_;
    };

    auto Data_Emitter::start_udp()->void
    {
        static int PORT_REMOTE =atoi(this->dest_port.c_str());// dest_port;
        static int PORT_HOST = atoi(this->local_port.c_str());//local_port;

        static const char* HOST_ADDR_STRING = "127.0.0.1";



        /* init udp_fd_sent*/
        if((this->udp_socket_fd = socket(AF_INET, SOCK_DGRAM,IPPROTO_UDP))<0)
        {
            perror("cannot create UDP socket");
        }
        else
        {
            printf("Create UDP socket.\n");
        }
        struct sockaddr_in myaddr;

        memset((char *)&this->host_addr_,0,sizeof(this->host_addr_));
        host_addr_.sin_family = AF_INET;
        host_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        host_addr_.sin_port = htons(PORT_HOST);
        if (bind(this->udp_socket_fd, (struct sockaddr *)&host_addr_, sizeof(host_addr_)) < 0)
        {
            perror("UDP bind failed");
            close(this->udp_socket_fd);
        }
        else
        {
            printf("UDP bind success.\n");
        }
        /*setup remote addr*/
        memset((char *)&this->remote_addr_,0,sizeof(this->remote_addr_));
        remote_addr_.sin_family = AF_INET;
//        remote_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        remote_addr_.sin_port = htons(PORT_REMOTE);
        inet_pton(AF_INET,this->dest_addr.c_str(),&remote_addr_.sin_addr);


    };

    auto Data_Emitter::close_udp()->void
    {
        close(this->udp_socket_fd);
    };

    auto Data_Emitter::sendto_udp(void *pdata, size_t length)->int
    {
        return sendto(this->udp_socket_fd,pdata,length,0,(struct sockaddr*)&remote_addr_,sizeof(remote_addr_));
    };

    auto Data_Emitter::recvfrom_udp(void *pdata, size_t length)->int
    {
       socklen_t addrlen=sizeof(remote_addr_);
       return recvfrom(this->udp_socket_fd,pdata,length,0,(struct sockaddr*)&remote_addr_,&addrlen);
    };

    }
#endif
		class EthercatMotion::Imp 
		{
		public:
			Imp(EthercatMotion *mot) :pFather(mot) {};
			~Imp() = default;

			std::int16_t enable(const std::uint8_t mode)
			{
				is_fake = false;				

				std::uint16_t statusWord;
				pFather->readPdo(1, 3, statusWord);

				std::uint8_t modeRead;
				pFather->readPdo(4, 0, modeRead);

				int motorState = (statusWord & 0x000F);

				if (motorState == 0x0000)
				{
					/*state is POWERED_OFF, now set it to STOPPED*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x06));
					return 1;
				}
				else if (motorState == 0x0001)
				{
					/*state is STOPPED, now set it to ENABLED*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x07));
					return 1;
				}
				else if (motorState == 0x0003)
				{
					/*state is ENABLED, now set it to RUNNING*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x0F));
					return 1;
				}
				else if ((mode == POSITION) && (modeRead != VELOCITY))
				{
					/*state is RUNNING, now to set desired mode*/
					/*desired mode is POSITION, but we need to use our own velocity loop*/
					pFather->writePdo(0, 5, static_cast<std::uint8_t>(VELOCITY));
					return 1;
				}
				else if ((mode != POSITION) && (modeRead != mode))
				{
					/*state is RUNNING, now change it to desired mode*/
					pFather->writePdo(0, 5, static_cast<std::uint8_t>(mode));
					return 1;
				}
				else if (motorState == 0x0007)
				{
					/*successfull, but still need to wait for 10 more cycles to make it stable*/
					switch (mode)
					{
					case POSITION:
					case VELOCITY:
						/*velocity loop to set velocity of 0*/
						pFather->writePdo(0, 1, static_cast<std::int32_t>(0));
						break;
					case CURRENT:
						pFather->writePdo(0, 2, static_cast<std::int16_t>(0));
						pFather->writePdo(0, 3, static_cast<std::int16_t>(1500));
						break;
					}

					if (++enable_period >= 10)
					{	
						running_mode = mode;
						enable_period = 0;
						return 0;
					}
					else
					{
						return 1;
					}
				}
				else
				{
					/*the motor is in fault*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x80));
					return 1;
				}
			}
			std::int16_t disable()
			{
				is_fake = false;					

				std::uint16_t statusWord;
				pFather->readPdo(1, 3, statusWord);

				int motorState = (statusWord & 0x000F);
				if (motorState == 0x0001)
				{
					/*alReady disabled*/
					return 0;
				}
				else if (motorState == 0x0003 || motorState == 0x0007 || motorState == 0x0000)
				{
					/*try to disable*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x06));
					return 1;
				}
				else
				{
					/*the motor is in fault*/
					pFather->writePdo(0, 4, static_cast<std::uint16_t>(0x80));
					return 1;
				}

			}
			std::int16_t home()
			{
				is_fake = false;					

				if(is_waiting_mode)
				{
					auto ret = this->enable(running_mode);
					is_waiting_mode = (ret == 0 ? false : true);
					return ret;
				}
				
				std::uint16_t statusWord;
				pFather->readPdo(1, 3, statusWord);
				int motorState = (statusWord & 0x000F);
				if (motorState != 0x0007)
				{
					return -1;
				}
				else
				{
					/*motor is in running state*/
					std::uint8_t mode_Read;
					pFather->readPdo(4, 0, mode_Read);
					if (mode_Read != 0x0006)
					{
						/*set motor to mode 0x006, which is homing mode*/
						pFather->writePdo(0, 5, static_cast<std::uint8_t>(0x006));
						return 1;
					}
					else
					{
						if (statusWord & 0x1000)
						{
							/*home finished, set mode to running mode, whose value is decided by 
							enable function, also write velocity to 0*/
							pFather->writePdo(0, 5, static_cast<uint8_t>(running_mode));
							is_waiting_mode = true;
							return 1;
						}
						else
						{
							/*still homing*/
							pFather->writePdo(0, 4, static_cast<uint16_t>(0x1F));
							return 1;
						}
					}
				}
			}
			std::int16_t runPos(const std::int32_t pos)
			{
				if (is_fake)return 0;

				std::uint16_t statusword;
				pFather->readPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->readPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != VELOCITY)
				{
					return -1;
				}
				else
				{
					std::int32_t current_pos = this->pos();
					std::int32_t desired_vel = static_cast<std::int32_t>((this->Kp_)*(pos - current_pos));

                    // prevent sudden jumping 
                    if (fabs((double)desired_vel) > 2.0 * fabs((double)max_vel_count_) ||
                        fabs((double)(pos - current_pos)) > fabs((double)max_vel_count_ * 2.0 / 1000.0))
                    {
                        rt_printf("Motor abs_id %d is encountering a sudden jumping, \
                                check your EtherCAT networking or planning program\n \
                                command pos: %d, current_pos: %d\n",
                                this->abs_id_, pos, current_pos);
                        return -1;
                    }
					
					/*保护上下限*/
					desired_vel = std::max(desired_vel, -max_vel_count_);
					desired_vel = std::min(desired_vel, max_vel_count_);
					
					pFather->writePdo(0, 1, desired_vel);
					return 0;
				}
			}
			std::int16_t runVel(const std::int32_t vel)
			{
				if (is_fake)return 0;

				std::uint16_t statusword;
				pFather->readPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->readPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != VELOCITY)
				{
					return -1;
				}
				else
				{
					pFather->writePdo(0, 1, vel);
					return 0;
				}
			}
			std::int16_t runCur(const std::int16_t cur)
			{
				if (is_fake)return 0;
								
				std::uint16_t statusword;
				pFather->readPdo(1, 3, statusword);
				int motorState = (statusword & 0x000F);

				std::uint8_t mode_Read;
				pFather->readPdo(4, 0, mode_Read);
				if (motorState != 0x0007 || mode_Read != CURRENT) //need running and cur mode
				{
					return -1;
				}
				else
				{
					pFather->writePdo(0, 2, cur);
					return 0;
				}
			}
			std::int32_t pos() { std::int32_t pos; pFather->readPdo(1, 0, pos); return pos + pos_offset_; };
			std::int32_t vel() { std::int32_t vel; pFather->readPdo(1, 2, vel); return vel; };
			std::int32_t cur() { std::int16_t cur; pFather->readPdo(2, 0, cur); return cur; };
            std::uint32_t dgi() { std::uint32_t dgi; pFather->readPdo(1, 1, dgi); return dgi; };
            std::uint16_t sta() { std::uint16_t sta; pFather->readPdo(1, 3, sta); return sta; };

            std::int32_t home_mode_;
			std::int32_t input2count_;
			std::int32_t home_count_;
			std::int32_t max_pos_count_;
			std::int32_t min_pos_count_;
			std::int32_t max_vel_count_;
			std::int32_t abs_id_;
			std::int32_t phy_id_;
            
            double Kp_{200};
			
			EthercatMotion *pFather;
			
			std::int32_t pos_offset_{0};

			bool is_fake{ true };
			bool is_waiting_mode{ false };
			
			int enable_period{ 0 };
			std::uint8_t running_mode{ 9 };

            bool is_home_with_switch_ {false};
            DIYHomingState homing_state_{DIYHomingState::READY};
            std::int32_t home_start_position_ { 0 };
            std::int32_t homing_wait_ticks_ { 0 };
		};

		EthercatMotion::~EthercatMotion() {}
		EthercatMotion::EthercatMotion(const aris::core::XmlElement &xml_ele, const aris::core::XmlElement &type_xml_ele) 
			:EthercatSlave(type_xml_ele), imp_(new EthercatMotion::Imp(this))
		{
			if (xml_ele.QueryIntAttribute("input2count", &imp_->input2count_) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"input2count\"");
			}

			double value;
			if (xml_ele.QueryDoubleAttribute("max_pos", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"max_pos\"");
			}
			imp_->max_pos_count_ = static_cast<std::int32_t>(value * imp_->input2count_);
			if (xml_ele.QueryDoubleAttribute("min_pos", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"min_pos\"");
			}
			imp_->min_pos_count_ = static_cast<std::int32_t>(value * imp_->input2count_);
			if (xml_ele.QueryDoubleAttribute("max_vel", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"max_vel\"");
			}
			imp_->max_vel_count_ = static_cast<std::int32_t>(value * imp_->input2count_);

			if (xml_ele.QueryDoubleAttribute("home_pos", &value) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"home_pos\"");
			}
			imp_->home_count_ = static_cast<std::int32_t>(value * imp_->input2count_);

			if (xml_ele.QueryIntAttribute("abs_id", &imp_->abs_id_) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find motion attribute \"abs_id\"");
			}

            // Assign specific kp gains for different types of motors if necessary
            double kp_gain = 200;
			if (xml_ele.QueryDoubleAttribute("kp", &kp_gain) != tinyxml2::XML_NO_ERROR)
			{
                imp_->Kp_ = 200;
			}
            else
            {
                imp_->Kp_ = kp_gain;
            }

            // Assign specific home mode for different types of motors if necessary
            int32_t hm_mode;
            int32_t hm_with_sw;
			if (xml_ele.QueryIntAttribute("sp_hm_mode", &hm_mode) == tinyxml2::XML_NO_ERROR)
			{
                imp_->home_mode_ = hm_mode;
                configSdo(0, static_cast<std::int8_t>(imp_->home_mode_));
                printf("Motor %d's home mode has been set to %d\n", imp_->phy_id_, imp_->home_mode_);
			}
            else if (xml_ele.QueryIntAttribute("hm_with_switch", &hm_with_sw) == tinyxml2::XML_NO_ERROR)
            {
                if (hm_with_sw) {
                    imp_->is_home_with_switch_ = true;
                    printf("Motor %d's home mode has been set to home with switch (DIY mode)\n", imp_->phy_id_);
                }
            }

			configSdo(9, static_cast<std::int32_t>(-imp_->home_count_));
		};
		auto EthercatMotion::writeCommand(const RawData &data)->void
		{		
			switch (data.cmd)
			{
			case IDLE:
				data.ret = 0;
				return;
			case ENABLE:
				data.ret = imp_->enable(data.mode);
				return;
			case DISABLE:
				data.ret = imp_->disable();
				return;
			case HOME:
				data.ret = imp_->home();
				return;
			case RUN:
				switch (data.mode)
				{
				case POSITION:
					data.ret = imp_->runPos(data.target_pos);
					return;
				case VELOCITY:
					data.ret = imp_->runVel(data.target_vel);
					return;
				case CURRENT:
					data.ret = imp_->runCur(data.target_cur);
					return;
				default:
					data.ret = -1;
					return;
				}
			default:
				data.ret = -1;
				return;
			}
		}
		auto EthercatMotion::readFeedback(RawData &data)->void
		{
			data.feedback_cur = imp_->cur();
			data.feedback_pos = imp_->pos();
			data.feedback_vel = imp_->vel();
            data.statusword = imp_->sta();
            data.feedback_dgi = imp_->dgi();
		}
		auto EthercatMotion::hasFault()->bool
		{
			std::uint16_t statusword;
			this->readPdo(1, 3, statusword);
			int motorState = (statusword & 0x000F);
			return (motorState != 0x0003 && motorState != 0x0007 && motorState != 0x0001 && motorState != 0x0000) ? true : false;
		}
		auto EthercatMotion::absID()->std::int32_t { return imp_->abs_id_; };
		auto EthercatMotion::phyID()->std::int32_t { return imp_->phy_id_; };
        auto EthercatMotion::home_count()->std::int32_t {return imp_->home_count_;};
		auto EthercatMotion::maxPosCount()->std::int32_t { return imp_->max_pos_count_; };
		auto EthercatMotion::minPosCount()->std::int32_t { return imp_->min_pos_count_; };
		auto EthercatMotion::maxVelCount()->std::int32_t { return imp_->max_vel_count_; };
		auto EthercatMotion::pos2countRatio()->std::int32_t { return imp_->input2count_; };
		auto EthercatMotion::setPosOffset(std::int32_t offset)->void
		{
			imp_->pos_offset_ = offset;
		};
		auto EthercatMotion::posOffset()const->std::int32_t
		{
			return imp_->pos_offset_;
		}

        auto EthercatMotion::is_home_with_switch() const -> bool
        {
            return imp_->is_home_with_switch_;
        }

        auto EthercatMotion::homing_state() -> EthercatMotion::DIYHomingState&
        {
            return imp_->homing_state_;
        }

        auto EthercatMotion::home_start_position() -> std::int32_t&
        {
            return imp_->home_start_position_;
        }

        auto EthercatMotion::homing_wait_ticks() -> std::int32_t&
        {
            return imp_->homing_wait_ticks_;
        }

		EthercatForceSensor::EthercatForceSensor(const aris::core::XmlElement &xml_ele, const aris::core::XmlElement &type_xml_ele) : EthercatSlave(type_xml_ele)
		{
			if (xml_ele.QueryIntAttribute("abs_id", &abs_id_) != tinyxml2::XML_NO_ERROR)
			{
				throw std::runtime_error("failed to find force sensor attribute \"abs_id\"");
			}
		};

		auto EthercatForceSensor::absID() -> std::int32_t
		{
			return abs_id_;
		}

		auto EthercatForceSensor::readData(Data &data)->void
		{
			std::int32_t value;

			this->readPdo(0, 0, value);
			raw_data_.Fx = static_cast<double>(value) / force_ratio_;

			this->readPdo(0, 1, value);
			raw_data_.Fy = static_cast<double>(value) / force_ratio_;

			this->readPdo(0, 2, value);
			raw_data_.Fz = static_cast<double>(value) / force_ratio_;

			this->readPdo(0, 3, value);
			raw_data_.Mx = static_cast<double>(value) / torque_ratio_;

			this->readPdo(0, 4, value);
			raw_data_.My = static_cast<double>(value) / torque_ratio_;

			this->readPdo(0, 5, value);
			raw_data_.Mz = static_cast<double>(value) / torque_ratio_;

            // if zeroing is required, average the raw readings and recalculate the base reading
            if (zeroing_count_left_ > 0)
            {
                zeroing_count_left_ --;
                for(int i = 0; i < 6; i++)
                {
                    sum_data_.fce[i] += raw_data_.fce[i];
                }
            }
            else if (zeroing_count_left_ == 0)
            {
                // update the base reading
                for(int i = 0; i < 6; i++)
                {
                    base_data_.fce[i] = sum_data_.fce[i]/ZEROING_COUNT;
                }
                zeroing_count_left_ --;
                rt_printf("Zeroing completed\n");
            }

            // return the force reading after substracting the base reading
            for(int i = 0; i < 6; i++)
            {
                data.fce[i] = raw_data_.fce[i] - base_data_.fce[i];
            }
		}

        auto EthercatForceSensor::requireZeroing() -> void
        {
            // if still in zeroing process, reject this request
            if (zeroing_count_left_ >= 0)
                return;

            // prepare to zero the reading of sensor
            for(auto& value : sum_data_.fce)
            {
                value = 0;
            }
            zeroing_count_left_ = ZEROING_COUNT;
            rt_printf("Zeroing started\n");
        }

		struct EthercatController::Imp
		{
            // map for motors
			std::vector<int> motor_map_phy2abs_;
            std::vector<int> motor_map_abs2phy_; 
            // map for force sensors
			std::vector<int> force_sensor_map_phy2abs_; 
            std::vector<int> force_sensor_map_abs2phy_; 

			std::function<int(Data&)> strategy_;
			Pipe<aris::core::Msg> msg_pipe_;
			std::atomic_bool is_stopping_;

			std::vector<EthercatMotion *> motion_vec_;
			std::vector<EthercatMotion::RawData> motion_rawdata_, last_motion_rawdata_;

			std::vector<EthercatForceSensor *> force_sensor_vec_;
			std::vector<EthercatForceSensor::Data> force_sensor_data_;

            std::vector<EthercatEK1100 *> slave_station_vec_;
            std::vector<EthercatEK1122 *> junction_vec_;
			std::thread record_thread_;
		};
		EthercatController::~EthercatController() {};
		EthercatController::EthercatController() :EthercatMaster(),imp_(new Imp) {};
		auto EthercatController::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			/*Load EtherCat slave types*/
			std::map<std::string, const aris::core::XmlElement *> slaveTypeMap;

			auto pSlaveTypes = xml_ele.FirstChildElement("SlaveType");
			for (auto type_xml_ele = pSlaveTypes->FirstChildElement(); type_xml_ele; type_xml_ele = type_xml_ele->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(type_xml_ele->name()), type_xml_ele));
			}

			/*Load all slaves*/
			imp_->motion_vec_.clear();
			imp_->force_sensor_vec_.clear();
			imp_->slave_station_vec_.clear();
			imp_->junction_vec_.clear();

			auto slave_xml = xml_ele.FirstChildElement("Slave");
			for (auto sla = slave_xml->FirstChildElement(); sla; sla = sla->NextSiblingElement())
			{
				std::string type{ sla->Attribute("type") };
                printf("Adding Slave as %s\n", type.c_str());
				if (type == "ElmoSoloWhistle")
				{
					imp_->motion_vec_.push_back(addSlave<EthercatMotion>(std::ref(*sla), std::ref(*slaveTypeMap.at(type))));
				}
				else if (type == "AtiForceSensor")
				{
					imp_->force_sensor_vec_.push_back(addSlave<EthercatForceSensor>(std::ref(*sla), std::ref(*slaveTypeMap.at(type))));
				}
				else if (type == "EK1100")
				{
					imp_->slave_station_vec_.push_back(addSlave<EthercatEK1100>(std::ref(*slaveTypeMap.at(type))));
				}
				else if (type == "EK1122")
				{
					imp_->junction_vec_.push_back(addSlave<EthercatEK1122>(std::ref(*slaveTypeMap.at(type))));
				}
				else
				{
					throw std::runtime_error(std::string("unknown slave type of \"") + type + "\"");
				}
			}

			/*update map for motors*/
			imp_->motor_map_phy2abs_.resize(imp_->motion_vec_.size());
			imp_->motor_map_abs2phy_.resize(imp_->motion_vec_.size());

			for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
			{
				imp_->motor_map_phy2abs_[i] = imp_->motion_vec_[i]->absID();
				motionAtPhy(i).imp_->phy_id_ = i;
			}

			for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
			{
				imp_->motor_map_abs2phy_[i] = 
                    std::find(imp_->motor_map_phy2abs_.begin(), imp_->motor_map_phy2abs_.end(), i)
                     - imp_->motor_map_phy2abs_.begin();
			}

            // update map for force sensors
			imp_->force_sensor_map_abs2phy_.resize(imp_->motion_vec_.size());
			imp_->force_sensor_map_phy2abs_.resize(imp_->motion_vec_.size());

			for (std::size_t i = 0; i < imp_->force_sensor_vec_.size(); ++i)
			{
				imp_->force_sensor_map_phy2abs_[i] = imp_->force_sensor_vec_[i]->absID();
			}
			for (std::size_t i = 0; i < imp_->force_sensor_vec_.size(); ++i)
			{
				imp_->force_sensor_map_abs2phy_[i] = 
                    std::find(imp_->force_sensor_map_phy2abs_.begin(), imp_->force_sensor_map_phy2abs_.end(), i)
                    - imp_->force_sensor_map_phy2abs_.begin();
			}

			/*resize other var*/
			imp_->motion_rawdata_.resize(imp_->motion_vec_.size());
			imp_->last_motion_rawdata_.resize(imp_->motion_vec_.size());
			imp_->force_sensor_data_.resize(imp_->force_sensor_vec_.size());
		}
		auto EthercatController::setControlStrategy(std::function<int(Data&)> strategy)->void
		{
			if (imp_->strategy_)
			{
				throw std::runtime_error("failed to set control strategy, because it alReady has one");
			}


			imp_->strategy_ = strategy;
		}
		auto EthercatController::start()->void
		{
			imp_->is_stopping_ = false;

			/*begin thread which will save data*/
			if(!imp_->record_thread_.joinable())
			imp_->record_thread_ = std::thread([this]()
			{
				static std::fstream file;

                std::string name = aris::core::logFileName();
                name.replace(name.rfind("log.txt"), std::strlen("data.txt"), "data.txt");
                file.open(name.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);

				std::vector<EthercatMotion::RawData> data;
				data.resize(imp_->motion_vec_.size());
#ifdef UNIX
                data_emitter::Data data_emitted;

                this->system_data_emitter.start_udp();
//                printf("Start UDP\n");
				int ret = 0;
#endif

				long long count = -1;
                
				while (!imp_->is_stopping_)
				{
#ifdef UNIX
                    ret=this->system_data_emitter.dataEmitterPipe().recvInNrt(data_emitted);
                    if(ret<0)
                    {
                        printf("Error in recvInNrt\n");
                    }
                    
                    if (count % 10 == 0) // We send data report to client at 20Hz
                    {
                        ret=this->system_data_emitter.sendto_udp(&data_emitted,sizeof(data_emitted));
                        if(ret<0)
                        {
                            //printf("Error in sendto_udp.\n");
                        }
                    }

                    if (isLog && count % 2 == 0)
                    { // We record data to file at 100Hz
                        file.write((char*)&data_emitted, sizeof(data_emitted));
                    }
#endif
                    count++;
				}

                file.close();

			});
			
			this->EthercatMaster::start();
		}
		auto EthercatController::stop()->void
		{
			this->EthercatMaster::stop();
		}
		auto EthercatController::motionNum()->std::size_t { return imp_->motion_vec_.size(); };
		auto EthercatController::motionAtAbs(int i)->EthercatMotion & { return *imp_->motion_vec_.at(imp_->motor_map_abs2phy_[i]); };
		auto EthercatController::motionAtPhy(int i)->EthercatMotion & { return *imp_->motion_vec_.at(i); };
		auto EthercatController::forceSensorNum()->std::size_t { return imp_->force_sensor_vec_.size(); };
		auto EthercatController::forceSensorAtAbs(int i)->EthercatForceSensor & { return *imp_->force_sensor_vec_.at(imp_->force_sensor_map_abs2phy_[i]); };
		auto EthercatController::forceSensorAtPhy(int i)->EthercatForceSensor & { return *imp_->force_sensor_vec_.at(i); };
		auto EthercatController::msgPipe()->Pipe<aris::core::Msg>& { return imp_->msg_pipe_; };
		auto EthercatController::controlStrategy()->void
		{
			/*构造传入strategy的参数*/
			Data data{ &imp_->last_motion_rawdata_, &imp_->motion_rawdata_, &imp_->force_sensor_data_, nullptr, nullptr };
			
			/*收取消息*/
			if (this->msgPipe().recvInRT(aris::core::MsgRT::instance[0]) > 0)
			{
				data.msg_recv = &aris::core::MsgRT::instance[0];
			};
			
			/*读取反馈*/
			for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
			{
				motionAtAbs(i).readFeedback(imp_->motion_rawdata_[i]);
			}
			for (std::size_t i = 0; i < imp_->force_sensor_vec_.size(); ++i)
			{
				forceSensorAtAbs(i).readData(imp_->force_sensor_data_[i]);
			}
			
			/*执行自定义的控制策略*/
			if (imp_->strategy_)
			{
				imp_->strategy_(data);
			}

			/*重新读取反馈信息，因为strategy可能修改已做好的反馈信息，之后写入PDO，之后放进lastMotionData中*/
			for (std::size_t i = 0; i < imp_->motion_rawdata_.size(); ++i)
			{
				motionAtAbs(i).readFeedback(imp_->motion_rawdata_[i]);
				motionAtAbs(i).writeCommand(imp_->motion_rawdata_[i]);
				imp_->last_motion_rawdata_[i] = imp_->motion_rawdata_[i];
			}

            // Handle the zeroing request from the strategy()
            for (std::size_t i = 0; i < imp_->force_sensor_data_.size(); ++i)
            {
                if (imp_->force_sensor_data_.at(i).isZeroingRequested)
                {
                    forceSensorAtAbs(i).requireZeroing();
                    imp_->force_sensor_data_.at(i).isZeroingRequested = false;
                }
            }

            /*Add another pipe for data distribution
              Or using the above pipe, but a socket for itself*/

			/*向外发送消息*/
			if (data.msg_send)
			{
				this->msgPipe().sendToNrt(*data.msg_send);
			}
		}
	}
}
