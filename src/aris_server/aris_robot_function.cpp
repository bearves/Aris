#include "aris_robot_function.h"

namespace aris
{
    namespace server
    {
        void BasicPlanner::setMotionSelector(const MotionSelector &selector)
        {
            motion_selector_ = selector;
        }

        ParseFunc BasicPlanner::basicParseFunc()
        {
            return [this](
                    const std::string &cmd, 
                    const std::map<std::string, std::string> &params,
                    aris::core::Msg &msg)
            {
                BasicFunctionParam param;
                motion_selector_(params, param);
                msg.copyStruct(param);
                return true;
            };
        }

        int BasicPlanner::enable(const BasicFunctionParam &param, 
                aris::control::EthercatController::Data &data,
                aris::control::EthercatController &controller)
        {
            bool is_all_enabled = true;

            for (std::size_t i = 0; i < data.motion_raw_data->size(); ++i)
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

                        if (param.count % controller.getControlFreq() == 0)
                        {
                            rt_printf("Unenabled motor, physical id: %d, absolute id: %d\n", controller.motionAtAbs(i).phyID(), i);
                        }
                    }
                }
            }

            return is_all_enabled ? 0 : 1;
        };

        int BasicPlanner::disable(const BasicFunctionParam &param, 
                aris::control::EthercatController::Data &data,
                aris::control::EthercatController &controller)
        {
            bool is_all_disabled = true;

            for (std::size_t i = 0; i < data.motion_raw_data->size(); ++i)
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

                        if (param.count % controller.getControlFreq() == 0)
                        {
                            rt_printf("Undisabled motor, physical id: %d, absolute id: %d\n", controller.motionAtAbs(i).phyID(), i);
                        }
                    }
                }
            }
            return is_all_disabled ? 0 : 1;
        }

        int BasicPlanner::home(const BasicFunctionParam &param, 
                aris::control::EthercatController::Data &data,
                aris::control::EthercatController &controller)
        {
            bool is_all_homed = true;

            for (std::size_t i = 0; i < data.motion_raw_data->size(); ++i)
            {
                if (param.active_motor[i])
                {
                    // 将电机的偏置置为0 //
                    controller.motionAtAbs(i).setPosOffset(0);

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

                        if (param.count % controller.getControlFreq() == 0)
                        {
                            rt_printf("Unhomed motor, physical id: %d, absolute id: %d\n", controller.motionAtAbs(i).phyID(), i);
                        }
                    }
                }
            }

            return is_all_homed ? 0 : 1;
        };


        int BasicPlanner::fake_home(const BasicFunctionParam &param, 
                aris::control::EthercatController::Data &data,
                aris::control::EthercatController &controller)
        {
            for (std::size_t i = 0; i < controller.motionNum(); ++i)
            {
                auto& motion = controller.motionAtAbs(i);
                motion.setPosOffset( static_cast<std::int32_t>(
                            motion.posOffset() + motion.homeCount() - data.motion_raw_data->at(i).feedback_pos
                            ));
            }

            for (std::size_t i = 0; i < controller.motionNum(); ++i){
                rt_printf("Pos2CountRatio %.3f\n", controller.motionAtAbs(i).pos2countRatio());
                rt_printf("Motpos: %.3f FeedbackPos: %d  posOffset: %d\n",
                        controller.motionAtAbs(i).homeCount(),
                        data.motion_raw_data->at(i).feedback_pos,
                        controller.motionAtAbs(i).posOffset()
                        );
            }

            return 0;
        };
        
        int BasicPlanner::zero_ruicong(const BasicFunctionParam &param, 
                aris::control::EthercatController::Data &data,
                aris::control::EthercatController &controller)
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

        int HomeSwitchPlanner::initializeStates(int motorNum)
        {
            motorNum_ = motorNum;
            motor_hmsw_states_.resize(motorNum);
            hmsw_state_count_.resize(motorNum);
            hmsw_stopping_vel_.resize(motorNum);
            return 0;
        }

        void HomeSwitchPlanner::setMotionSelector(const MotionSelector &selector)
        {
            motion_selector_ = selector;
        }

        int HomeSwitchPlanner::hmsw(const HmswFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller)
        {
            using aris::control::EthercatMotion;
            double dT = 1.0 / controller.getControlFreq();

            bool is_all_hmsw_finished = true;

            for (std::size_t i = 0; i < data.motion_raw_data->size(); ++i)
            {
                if (param.active_motor[i])
                {
                    if (param.count == 0)
                    {
                        // initalize hmsw states for each motor
                        motor_hmsw_states_[i] = HMSW_STATE::NOT_READY;
                        hmsw_state_count_[i] = param.count+1;
                        is_all_hmsw_finished = false;
                    }
                    else if (motor_hmsw_states_[i] == HMSW_STATE::NOT_READY)
                    {
                        // check if this motor have been enabled with desired mode
                        if (param.count != hmsw_state_count_[i] && data.motion_raw_data->operator[](i).ret == 0)
                        {
                            motor_hmsw_states_[i] = HMSW_STATE::HOMING;
                            hmsw_state_count_[i] = param.count+1;
                        }
                        else{
                            // enable this motor and switch it to VELOCITY mode
                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::ENABLE;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::VELOCITY;
                            data.motion_raw_data->operator[](i).target_vel = 0;
                        }
                        is_all_hmsw_finished = false;
                    }
                    else if (motor_hmsw_states_[i] == HMSW_STATE::HOMING)
                    {
                        // check if the home switch is reached
                        int home_switch_reached = (data.motion_raw_data->operator[](i).digital_inputs & (1<<20)) >> 20;
                        if (home_switch_reached == 1)
                        {
                            motor_hmsw_states_[i] = HMSW_STATE::SWITCH_REACHED;
                            hmsw_state_count_[i] = param.count+1;
                            hmsw_stopping_vel_[i] = data.motion_raw_data->operator[](i).feedback_vel;
                        }
                        else
                        {
                            int desired_vel = param.hmsw_velocity_in_count;
                            double hm_acc = param.hmsw_accel_in_count;

                            // avoid singularity
                            if (fabs(hm_acc) < 1)
                                hm_acc = HMSW_DEFAULT_ACCEL;

                            // make hm_acc the same sign with the vel
                            if (desired_vel < 0)
                                hm_acc = -fabs(hm_acc);

                            // move motor 
                            int homing_count = param.count - hmsw_state_count_[i];
                            if (homing_count < fabs(desired_vel / hm_acc)/dT)
                            {
                                // accel motor
                                data.motion_raw_data->operator[](i).target_vel = (int)(hm_acc * dT * homing_count);
                            }
                            else
                            {
                                // constant speed homing
                                data.motion_raw_data->operator[](i).target_vel = desired_vel;
                            }

                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::VELOCITY;
                        }
                        is_all_hmsw_finished = false;
                    }
                    else if (motor_hmsw_states_[i] == HMSW_STATE::SWITCH_REACHED)
                    {
                        double hm_dec = param.hmsw_accel_in_count;
                        if (fabs(hm_dec) < 1) 
                            hm_dec = HMSW_DEFAULT_ACCEL;

                        // make hm_acc the same sign with the vel
                        if (hmsw_stopping_vel_[i] < 0)
                            hm_dec = -fabs(hm_dec);

                        int deccel_count = fabs(hmsw_stopping_vel_[i] / hm_dec)/dT;
                        //check if the motor comes to a stop
                        if (param.count > (hmsw_state_count_[i]+deccel_count + 1))
                        {
                            motor_hmsw_states_[i] = HMSW_STATE::SETTING_OFFSET;
                            hmsw_state_count_[i] = param.count+1;
                        }
                        else
                        {
                            int stopping_count = param.count - hmsw_state_count_[i];
                            // deccel motor 
                            if (stopping_count < deccel_count)
                            {
                                data.motion_raw_data->operator[](i).target_vel = 
                                    (int)(hmsw_stopping_vel_[i] - hm_dec * dT * stopping_count);
                            }
                            else
                            {
                                data.motion_raw_data->operator[](i).target_vel = 0;
                            }
                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::VELOCITY;
                        }

                        is_all_hmsw_finished = false;
                    }
                    else if (motor_hmsw_states_[i] == HMSW_STATE::SETTING_OFFSET)
                    {
                        // after setting new posOffset, we should wait a few counts for
                        // new feedback_pos to be updated
                        if (param.count > (hmsw_state_count_[i] + 2))
                        {
                            motor_hmsw_states_[i] = HMSW_STATE::HOME_FINISHED;
                            hmsw_state_count_[i] = param.count+1;
                        }
                        else
                        {
                            if (param.count == hmsw_state_count_[i])
                            {
                                // ask motor to set new posOffset at home
                                auto& motion = controller.motionAtAbs(i);
                                motion.setPosOffset( static_cast<std::int32_t>(
                                            motion.posOffset() + motion.homeCount() -
                                            data.motion_raw_data->at(i).feedback_pos
                                            ));
                            }
                        }
                        is_all_hmsw_finished = false;
                    }
                    else // the motor has reached the HMSW_STATE::HOME_FINISHED state
                    {
                        if (param.count == hmsw_state_count_[i])
                        {
                            // switch motor back to the POSITION mode
                            data.motion_raw_data->operator[](i).cmd = EthercatMotion::RUN;
                            data.motion_raw_data->operator[](i).mode = EthercatMotion::POSITION;
                            data.motion_raw_data->operator[](i).target_pos = data.motion_raw_data->operator[](i).feedback_pos;
                            data.motion_raw_data->operator[](i).target_vel = 0;
                            data.motion_raw_data->operator[](i).target_cur = 0;
                            rt_printf("Motor %d home finished\n", i);
                        }
                    }
                }
            }

            return is_all_hmsw_finished ? 0 : 1;
        }

        ParseFunc HomeSwitchPlanner::hmswCmdParseFunc()
        {
            return [this](
                    const std::string &cmd, 
                    const std::map<std::string, std::string> &params,
                    aris::core::Msg &msg)
            {
                HmswFunctionParam param;
                motion_selector_(params, param);

                for (auto i : params)
                {
                    if (i.first == "vel")
                    {
                        param.hmsw_velocity_in_count = std::stoi(i.second);
                    }
                    else if (i.first == "acc")
                    {
                        param.hmsw_accel_in_count = std::stoi(i.second);
                    }
                }
                msg.copyStruct(param);
                return true;
            };
        }

        int JogPlanner::initializeStates(int motorNum)
        {
            motorNum_ = motorNum;
            motor_jog_states_.resize(motorNum);
            jog_state_count_.resize(motorNum);
            jog_stopping_vel_.resize(motorNum);
            return 0;
        }

        void JogPlanner::setMotionSelector(const MotionSelector &selector)
        {
            motion_selector_ = selector;
        }

        int JogPlanner::jog(const JogFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller)
        {
            using aris::control::EthercatMotion;
            double dT = 1.0 / controller.getControlFreq();

            bool is_all_jog_finished = true;

            for (std::size_t i = 0; i < data.motion_raw_data->size(); ++i)
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
                        bool time_limit_reached = (param.count > (jog_state_count_[i] + JOG_TIME_LIMIT_SECOND * controller.getControlFreq()));
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
                            if (jogging_count < fabs(desired_vel / jog_acc) / dT)
                            {
                                // accel motor
                                data.motion_raw_data->operator[](i).target_vel = (int)(jog_acc * dT * jogging_count);
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

                        int deccel_count = fabs(jog_stopping_vel_[i] / jog_dec) / dT;
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
                                    (int)(jog_stopping_vel_[i] - jog_dec * dT * stopping_count);
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

        ParseFunc JogPlanner::jogCmdParseFunc()
        {
            return [this](
                    const std::string &cmd, 
                    const std::map<std::string, std::string> &params,
                    aris::core::Msg &msg)
            {
                JogFunctionParam param;
                motion_selector_(params, param);

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

                jog_stop_signal_received_ = param.requireStop;
                bool queue_flag = !param.requireStop;

                msg.copyStruct(param);
                return queue_flag;
            };
        }
    }


}
