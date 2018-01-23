#ifndef ARIS_ROBOT_FUNCTION_H
#define ARIS_ROBOT_FUNCTION_H

#include <map>
#include <aris_core.h>
#include <aris_control.h>
#include <aris_model.h>
#include <rtdk.h>

namespace aris
{
    namespace server
    {
        enum { MAX_MOTOR_NUM = 100 };
        enum { MAX_FSR_CHANNEL_NUM = 20 };

        struct PlanParamBase
        {
            std::int32_t cmd_type{ 0 };
            mutable std::int32_t count{ 0 };
        };

        //for enable, disable, and home
        struct BasicFunctionParam : PlanParamBase
        {
            public:
                bool active_motor[MAX_MOTOR_NUM];
                bool active_channel[MAX_FSR_CHANNEL_NUM];

                BasicFunctionParam() 
                { 
                    std::fill(active_motor, active_motor + MAX_MOTOR_NUM, true);
                    std::fill(active_channel, active_channel + MAX_FSR_CHANNEL_NUM, false);
                };
        };

        // for hmsw
        struct HmswFunctionParam : BasicFunctionParam
        {
            public:
                int hmsw_velocity_in_count;
                int hmsw_accel_in_count;
        };

        // for jog 
        struct JogFunctionParam : BasicFunctionParam
        {
            public:
                int jog_velocity_in_count;
                int jog_accel_in_count;
                bool requireStop;
        };

        //for all ordinary gaits
        struct GaitParamBase :BasicFunctionParam
        {
            bool if_check_pos_min{ true };
            bool if_check_pos_max{ true };
            bool if_check_pos_continuous{ true };
            std::int32_t gait_id;
            //            const aris::sensor::ImuData *imu_data;
            const std::vector<aris::control::EthercatForceSensor::Data> *force_data;
            std::vector<aris::control::EthercatForceSensorRuiCongCombo::RuiCongComboData> *ruicong_data;
            const std::vector<aris::control::EthercatIMU::Data> *imu_data;
            std::vector<aris::control::EthercatMotion::RawData> *motion_raw_data;
            const std::vector<aris::control::EthercatMotion::RawData> *last_motion_raw_data;
            const std::vector<double> *motion_feedback_pos;
        };

        typedef std::function<void(const std::map<std::string, std::string> &dom, aris::server::BasicFunctionParam &param)> MotionSelector;
        typedef std::function<bool(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)> ParseFunc;
        typedef std::function<int(aris::model::Model &, PlanParamBase &)> PlanFunc;

        class BasicPlanner
        {
            public:
                void setMotionSelector(const MotionSelector &selector);
                ParseFunc basicParseFunc();

                int enable(const BasicFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller);

                int disable(const BasicFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller);

                int home(const BasicFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller);

                int fake_home(const BasicFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller);

                int zero_ruicong(const BasicFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller);

            private:
                MotionSelector motion_selector_;
        };

        class HomeSwitchPlanner
        {
            public:
                // internal states for home by switch
                enum HMSW_STATE
                {
                    NOT_READY       = 0,
                    HOMING          = 1,
                    SWITCH_REACHED  = 2,
                    SETTING_OFFSET  = 3,
                    HOME_FINISHED   = 4
                };

                int initializeStates(int motorNum);
                void setMotionSelector(const MotionSelector &selector);
                ParseFunc hmswCmdParseFunc();
                int hmsw(const HmswFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller);

            private:
                std::size_t motorNum_;
                const double HMSW_DEFAULT_ACCEL = 200000;
                const double HMSW_DEFAULT_VEL   = 20000;
                std::vector<HMSW_STATE> motor_hmsw_states_;
                std::vector<std::size_t> hmsw_state_count_;
                std::vector<int> hmsw_stopping_vel_;
                MotionSelector motion_selector_;
        };

        class JogPlanner
        {
            public:
                // internal states for jogging
                enum JOG_STATE
                {
                    NOT_READY = 0,
                    JOGGING   = 1,
                    STOPPING  = 2,
                    STOPPED   = 3
                };

                int initializeStates(int motorNum);
                void setMotionSelector(const MotionSelector &selector);
                ParseFunc jogCmdParseFunc();
                int jog(const JogFunctionParam &param, 
                        aris::control::EthercatController::Data &data,
                        aris::control::EthercatController &controller);

            private:
                std::size_t motorNum_;
                std::atomic_bool jog_stop_signal_received_{false};
                const double JOG_DEFAULT_ACCEL = 160000;
                const int JOG_TIME_LIMIT_SECOND = 10;
                std::vector<JOG_STATE> motor_jog_states_;
                std::vector<std::size_t> jog_state_count_;
                std::vector<int> jog_stopping_vel_;
                MotionSelector motion_selector_;
        };
    }
}

#endif
