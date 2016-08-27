#ifndef ARIS_CONTROL_MOTION_H
#define ARIS_CONTROL_MOTION_H

#include <functional>
#include <thread>
#include <atomic>
#include <array>
#include <string>

#ifdef UNIX
// date_emitter
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <aris_control_ethercat.h>
#include <aris_sensor_imu.h>


namespace aris
{
	namespace control
	{	
		class EthercatMotion :public EthercatSlave
		{
		public:
			enum Cmd
			{
				IDLE = 0,
				ENABLE,
				DISABLE,
				HOME,
				RUN
			};
			enum Mode
			{
				POSITION = 0x0008,
				VELOCITY = 0x0009,
				CURRENT = 0x0010,
			};

			// only used for DIY homing
			enum class DIYHomingState
			{
				READY   = 0,
				HOMING  = 1,
				HOMED   = 2,
				RESTORE = 3
			};

			struct RawData
			{
				std::int32_t target_pos{ 0 }, feedback_pos{ 0 };
				std::int32_t target_vel{ 0 }, feedback_vel{ 0 };
				std::int16_t target_cur{ 0 }, feedback_cur{ 0 };
				std::uint8_t cmd{ IDLE };
				std::uint8_t mode{ POSITION };
				std::uint16_t statusword{ 0 };
				std::int32_t feedback_dgi{ 0 };
				mutable std::int16_t ret{ 0 };

				inline bool is_home_switch_on()
				{
					return (feedback_dgi & 0x00300000) == 0x00200000;
				};
			};

			virtual ~EthercatMotion();
			EthercatMotion(const aris::core::XmlElement &xml_ele, const aris::core::XmlElement &type_xml_ele);
			auto hasFault()->bool;
			auto readFeedback(RawData &data)->void;
			auto writeCommand(const RawData &data)->void;
			auto absID()->std::int32_t;
			auto phyID()->std::int32_t;
			auto maxPosCount()->std::int32_t;
			auto minPosCount()->std::int32_t;

			auto maxVelCount()->std::int32_t;
			auto pos2countRatio()->std::int32_t;
			auto setPosOffset(std::int32_t offset)->void;
			auto posOffset()const->std::int32_t;

			// for DIY homing mode with homing switch
			auto is_home_with_switch() const -> bool;
			auto homing_state() -> DIYHomingState&;
			auto home_start_position() -> std::int32_t&;
			auto homing_wait_ticks() -> std::int32_t&;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;

			friend class EthercatController;
		};
		class EthercatForceSensor final:public EthercatSlave
		{
		public:
			struct Data
			{
				union
				{
					struct { double Fx, Fy, Fz, Mx, My, Mz; };
					double fce[6];
				};
				bool isZeroingRequested;
			};

			EthercatForceSensor(const aris::core::XmlElement &xml_ele, const aris::core::XmlElement &type_xml_ele);

			auto absID() -> std::int32_t;

			auto readData(Data &data)->void;
			auto requireZeroing() -> void;

		protected:
			static const int ZEROING_COUNT = 500;
			virtual auto init()->void override
			{
				this->readSdo(0, force_ratio_);
				this->readSdo(1, torque_ratio_);
				this->zeroing_count_left_ = -1;

				for(auto value : base_data_.fce)
				{
					value = 0;
				}
			};
			std::int32_t abs_id_;
			std::int32_t force_ratio_, torque_ratio_;
			EthercatForceSensor::Data base_data_;
			EthercatForceSensor::Data sum_data_;
			EthercatForceSensor::Data raw_data_;
			std::int32_t zeroing_count_left_;
		};

        // Beckhoff Slave station 
        class EthercatEK1100 final: public EthercatSlave
        {
        public:
			EthercatEK1100(const aris::core::XmlElement &xml_ele): EthercatSlave(xml_ele){};
        };

        // Beckhoff slave 1 to 2 junction
        class EthercatEK1122 final: public EthercatSlave
        {
        public:
			EthercatEK1122(const aris::core::XmlElement &xml_ele): EthercatSlave(xml_ele){};
        };

        /*all things needed for emitting data to the outside*/
        namespace data_emitter
        {
        struct ForceDataCompact
        {
            float Fx, Fy, Fz, Mx, My, Mz;
        };
        struct IMUDataCompact
        {
            float yaw,pitch,roll;
        };

        struct Data
        {
            static const int MOT_NUM=19;
            static const int FOR_NUM=7;
            int32_t timecount;
            IMUDataCompact imu_data;
            std::array<aris::control::EthercatMotion::RawData,MOT_NUM> motor_data;
            std::array<ForceDataCompact,FOR_NUM> force_data;
            //aris::sensor::ImuData imu_data;
        };
#ifdef UNIX
        /*Keep it simple and stupid*/
        class Data_Emitter
        {
        public:
            auto setUDP(std::string d_addr,std::string d_port,std::string l_port)->void;
            auto dataEmitterPipe()->aris::control::Pipe<Data>&;
            auto start_udp()->void;
            auto close_udp()->void;
            auto sendto_udp(void* pdata, size_t length)->int;
            auto recvfrom_udp(void* pdata, size_t length)->int;

        private:

            static const int BUFF_SIZE=8192;
            aris::control::Pipe<Data> data_emitter_pipe_;
            int udp_socket_fd;
            int udp_fd_recv_;
            char sent_buff_[BUFF_SIZE];
            char recv_buff_[BUFF_SIZE];

            struct sockaddr_in remote_addr_;
            struct sockaddr_in host_addr_;

            std::string dest_addr,dest_port,local_port;
        };
#endif
        }//namespace data_emitter



		class EthercatController :public EthercatMaster
		{
		public:
			struct Data
			{
				const std::vector<EthercatMotion::RawData> *last_motion_raw_data;
				std::vector<EthercatMotion::RawData> *motion_raw_data;
				std::vector<EthercatForceSensor::Data> *force_sensor_data;
				const aris::core::MsgRT *msg_recv;
				aris::core::MsgRT *msg_send;
			};

			virtual ~EthercatController();
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void override;
			virtual auto start()->void;
			virtual auto stop()->void;
			auto setControlStrategy(std::function<int(Data&)>)->void;
			auto motionNum()->std::size_t;
			auto motionAtAbs(int i)->EthercatMotion &;
			auto motionAtPhy(int i)->EthercatMotion &;
			auto forceSensorNum()->std::size_t;
			auto forceSensorAtPhy(int i)->EthercatForceSensor &;
			auto forceSensorAtAbs(int i)->EthercatForceSensor &;
			auto msgPipe()->Pipe<aris::core::Msg>&;

#ifdef UNIX
            data_emitter::Data_Emitter system_data_emitter;
            data_emitter::Data data_emitter_data_;
#endif
            bool isLog=true;

		protected:
			EthercatController();
			virtual auto controlStrategy()->void override final;

		private:
			struct Imp;
			std::unique_ptr<Imp> imp_;

			friend class EthercatMaster;

		};


    }
}

#endif
