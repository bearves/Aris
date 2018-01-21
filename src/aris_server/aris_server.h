#ifndef ARIS_SERVER_H
#define ARIS_SERVER_H

#include <string>
#include <sstream>
#include <map>
#include <memory>

#include <aris_core.h>
#include <aris_control.h>
#include <aris_model.h>
#include <aris_node.h>
#include "aris_robot_function.h"

namespace aris
{
	namespace server
	{
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

