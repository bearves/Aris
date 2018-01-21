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
            const std::vector<aris::control::EthercatMotion::RawData> *motion_raw_data;
            const std::vector<aris::control::EthercatMotion::RawData> *last_motion_raw_data;
            const std::vector<double> *motion_feedback_pos;
        };

        typedef std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)> ParseFunc;
        typedef std::function<int(aris::model::Model &, const PlanParamBase &)> PlanFunc;

        class JogPlanner
        {
            public:
                // internal states for jog gait
                enum JOG_STATE
                {
                    NOT_READY = 0,
                    JOGGING   = 1,
                    STOPPING  = 2,
                    STOPPED   = 3
                };

                int initialize(int motorNum);
                ParseFunc jogCmdParseFunc();
                bool jogCmdPostProcess(const JogFunctionParam &param);
                int jog(const JogFunctionParam &param, aris::control::EthercatController::Data &data);

            private:
                std::size_t motorNum_;
                std::atomic_bool jog_stop_signal_received_{false};
                const double JOG_DEFAULT_ACCEL = 160000;
                const int JOG_TIME_LIMIT_COUNT = 10000;
                std::vector<JOG_STATE> motor_jog_states_;
                std::vector<std::size_t> jog_state_count_;
                std::vector<int> jog_stopping_vel_;
        };
    }
}

#endif
