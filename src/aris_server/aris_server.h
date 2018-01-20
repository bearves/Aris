﻿#ifndef ARIS_SERVER_H
#define ARIS_SERVER_H

#include <string>
#include <sstream>
#include <map>
#include <memory>

#include <aris_core.h>
#include <aris_control.h>
#include <aris_model.h>
#include <aris_node.h>

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

		class ControlServer
		{
		public:
			static ControlServer &instance();

			template<typename T>
			auto createModel()->void { this->createModel(new T); };
			auto createModel(model::Model *model)->void;

			auto loadXml(const char *fileName)->void;
			auto loadXml(const aris::core::XmlDocument &xmlDoc)->void;
			auto model()->model::Model&;
			auto controller()->control::EthercatController&;
			auto addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const PlanFunc &gait_func)->void;
			auto open()->void;
			auto close()->void;
			auto setOnExit(std::function<void(void)> callback_func)->void;

		private:
			~ControlServer();
			ControlServer();
			ControlServer(const ControlServer &) = delete;
			ControlServer &operator=(const ControlServer &) = delete;

		private:
			class Imp;
			std::unique_ptr<Imp> imp;
		};
	}
}

#endif

