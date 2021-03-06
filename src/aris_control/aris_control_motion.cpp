﻿#ifdef UNIX
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
#include <memory>
#include <algorithm>
#include "io_mapping_definition.h"

#include "aris_control_motion.h"


namespace aris
{
    namespace control
    {
        class EthercatMotion::Imp 
        {
            public:
                Imp(EthercatMotion *mot, EthercatDriver *driver) :pFather(mot), pdrv(driver) {};
                ~Imp() = default;

                std::int32_t SetIOMapping(const aris::core::XmlElement &xml_ele)
                {
                    std::string type_name{ xml_ele.Attribute("type") };
                    type_ = type_name;
                    if (type_ == "ElmoSoloWhistle")
                    {
                        io_mapping_.reset(new ElmoGuitarDef());
                    }
                    else if (type_ == "Faulhaber")
                    {
                        io_mapping_.reset(new FaulhaberDef());
                    }
                    else if (type_ == "CopleyAE2_A")
                    {
                        io_mapping_.reset(new CopleyAE2Def_A());
                    }
                    else if (type_ == "CopleyAE2_B")
                    {
                        io_mapping_.reset(new CopleyAE2Def_B());
                    }
                    std::cout << "The io mapping is set to " << type_ << std::endl;
                    return 0;
                }

                std::int16_t enable(const std::uint8_t mode)
                {
                    is_fake = false;				

                    std::uint16_t statusWord = this->statusWord();

                    std::uint8_t modeRead = this->operationMode();
                    static std::int32_t current_pos;

                    int motorState = (statusWord & 0x000F);

                    /* prepare target pos/vel/trq to the current position/0/0 in any cases */
                    current_pos = this->pos();

                    pdrv->writePdo(
                            io_mapping_->targetPosition_index, 
                            io_mapping_->targetPosition_subindex, 
                            current_pos - pos_offset_);

                    /* set velocity and torque to 0*/
                    pdrv->writePdo(
                            io_mapping_->targetVelocity_index, 
                            io_mapping_->targetVelocity_subindex, 
                            static_cast<std::int32_t>(0));
                    pdrv->writePdo(
                            io_mapping_->targetTorque_index, 
                            io_mapping_->targetTorque_subindex, 
                            static_cast<std::int16_t>(0));

                    if (motorState == 0x0000)
                    {
                        /*state is POWERED_OFF, now set it to STOPPED*/
                        pdrv->writePdo(
                                io_mapping_->controlWord_index, 
                                io_mapping_->controlWord_subindex, 
                                static_cast<std::uint16_t>(0x06));
                        return 1;
                    }
                    else if (motorState == 0x0001)
                    {
                        /*state is STOPPED, now set it to ENABLED*/
                        pdrv->writePdo(
                                io_mapping_->controlWord_index, 
                                io_mapping_->controlWord_subindex, 
                                static_cast<std::uint16_t>(0x07));
                        return 1;
                    }
                    else if (motorState == 0x0003)
                    {
                        /*state is ENABLED, now set it to RUNNING*/
                        pdrv->writePdo(
                                io_mapping_->controlWord_index, 
                                io_mapping_->controlWord_subindex, 
                                static_cast<std::uint16_t>(0x0F));
                        return 1;
                    }
                    else if (motorState == 0x0007)
                    {

                        /*state is RUNNING, now change it to desired mode*/
                        if (modeRead != mode)
                        {
                            pdrv->writePdo(
                                    io_mapping_->modeOfOperation_index, 
                                    io_mapping_->modeOfOperation_subindex, 
                                    static_cast<std::uint8_t>(mode));
                            return 1;
                        }

                        /*successfull, but still need to wait for 10 more cycles to make it stable*/
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
                        pdrv->writePdo(
                                io_mapping_->controlWord_index, 
                                io_mapping_->controlWord_subindex, 
                                static_cast<std::uint16_t>(0x80));
                        return 1;
                    }
                }
                std::int16_t disable()
                {
                    is_fake = false;					

                    std::uint16_t statusWord = this->statusWord();

                    int motorState = (statusWord & 0x000F);
                    if (motorState == 0x0001)
                    {
                        /*alReady disabled*/
                        return 0;
                    }
                    else if (motorState == 0x0003 || motorState == 0x0007 || motorState == 0x0000)
                    {
                        /*try to disable*/
                        pdrv->writePdo(
                                io_mapping_->controlWord_index, 
                                io_mapping_->controlWord_subindex, 
                                static_cast<std::uint16_t>(0x06));
                        return 1;
                    }
                    else
                    {
                        /*the motor is in fault*/
                        pdrv->writePdo(
                                io_mapping_->controlWord_index, 
                                io_mapping_->controlWord_subindex, 
                                static_cast<std::uint16_t>(0x80));
                        return 1;
                    }

                }
                std::int16_t home()
                {
                    is_fake = false;					

                    if(is_waiting_mode)
                    {
                        // wait for 25 counts preparing the mode-switch
                        // from HOMING to RUNNING
                        if (waiting_count < 25)
                        {
                            std::int32_t current_pos = this->pos();

                            pdrv->writePdo(
                                    io_mapping_->targetPosition_index, 
                                    io_mapping_->targetPosition_subindex, 
                                    current_pos - pos_offset_);

                            /* set velocity and torque to 0*/
                            pdrv->writePdo(
                                    io_mapping_->targetVelocity_index, 
                                    io_mapping_->targetVelocity_subindex, 
                                    static_cast<std::int32_t>(0));
                            pdrv->writePdo(
                                    io_mapping_->targetTorque_index, 
                                    io_mapping_->targetTorque_subindex, 
                                    static_cast<std::int16_t>(0));

                            pdrv->writePdo(
                                    io_mapping_->controlWord_index, 
                                    io_mapping_->controlWord_subindex,
                                    static_cast<uint16_t>(0x0F));

                            waiting_count++;
                            return 1;
                        }
                        else{
                            auto ret = this->enable(running_mode);
                            is_waiting_mode = (ret == 0 ? false : true);
                            return ret;
                        }
                    }

                    std::uint16_t statusWord = this->statusWord();
                    std::uint8_t mode_Read = this->operationMode();
                    // get current status and op mode
                    int motorState = (statusWord & 0x000F);

                    if (motorState == 0x0007 && mode_Read != HOMING)
                    {
                        // jump out from other modes
                        pdrv->writePdo(
                                io_mapping_->controlWord_index, 
                                io_mapping_->controlWord_subindex,
                                static_cast<uint16_t>(0x06));
                        return 1;
                    }
                    else if (mode_Read != HOMING) // motorState is not running but the mode have not changed to home mode yet
                    {
                        pdrv->writePdo(
                                io_mapping_->modeOfOperation_index, 
                                io_mapping_->modeOfOperation_subindex, 
                                static_cast<std::uint8_t>(HOMING));
                        return 1;
                    }
                    else if (motorState != 0x0007) // mode must have changed but the motor state is not running yet
                    {
                        // now enable the motor to the HOMING mode
                        if (motorState == 0x0003)
                        {
                            /*state is ENABLED, now set it to RUNNING*/
                            pdrv->writePdo(
                                    io_mapping_->controlWord_index, 
                                    io_mapping_->controlWord_subindex, 
                                    static_cast<std::uint16_t>(0x0F));
                        }
                        else if (motorState == 0x0001)
                        {
                            pdrv->writePdo(
                                    io_mapping_->controlWord_index, 
                                    io_mapping_->controlWord_subindex, 
                                    static_cast<std::uint16_t>(0x07));
                        }
                        else if (motorState == 0x0000)
                        {
                            pdrv->writePdo(
                                    io_mapping_->controlWord_index, 
                                    io_mapping_->controlWord_subindex, 
                                    static_cast<std::uint16_t>(0x06));
                        }

                        pdrv->writePdo(
                                io_mapping_->modeOfOperation_index, 
                                io_mapping_->modeOfOperation_subindex, 
                                static_cast<std::uint8_t>(HOMING));

                        home_period = 0;
                        return 1;
                    }
                    else
                    {
                        /*motor is in running state and in homing mode*/

                        if (statusWord & 0x1000)
                        {
                            /*home finished, set mode to running mode, whose value is decided by 
                              enable function, also write velocity to 0*/
                            pdrv->writePdo(
                                    io_mapping_->controlWord_index, 
                                    io_mapping_->controlWord_subindex,
                                    static_cast<uint16_t>(0x1F));

                            waiting_count = 0;
                            is_waiting_mode = true;
                            return 1;
                        }
                        else
                        {
                            // prepare to start homing
                            if (home_period < 25)
                            {
                                pdrv->writePdo(
                                        io_mapping_->controlWord_index, 
                                        io_mapping_->controlWord_subindex,
                                        static_cast<uint16_t>(0x0F));
                                home_period++;
                                return 1;
                            }
                            // homing have started
                            /*still homing*/
                            pdrv->writePdo(
                                    io_mapping_->controlWord_index, 
                                    io_mapping_->controlWord_subindex,
                                    static_cast<uint16_t>(0x1F));

                            return 1;
                        }

                    }
                }
                std::int16_t runPos(const std::int32_t pos)
                {
                    if (is_fake)
                        return 0;

                    std::uint16_t statusword = this->statusWord();

                    int motorState = (statusword & 0x000F);

                    std::uint8_t mode_Read = this->operationMode();

                    if (motorState != 0x0007 || mode_Read != POSITION)
                    {
                        return -1;
                    }
                    else
                    {
                        std::int32_t current_pos = this->pos();

                        pdrv->writePdo(
                                io_mapping_->targetPosition_index, 
                                io_mapping_->targetPosition_subindex, 
                                pos - pos_offset_);

                        return 0;
                    }
                }

                std::int16_t runVel(const std::int32_t vel)
                {
                    if (is_fake)
                        return 0;

                    std::uint16_t statusword = this->statusWord();
                    int motorState = (statusword & 0x000F);

                    std::uint8_t mode_Read = this->operationMode();

                    if (motorState != 0x0007 || mode_Read != VELOCITY)
                    {
                        return -1;
                    }
                    else
                    {
                        pdrv->writePdo(
                                io_mapping_->targetVelocity_index, 
                                io_mapping_->targetVelocity_subindex,
                                vel);
                        return 0;
                    }
                }

                std::int16_t runCur(const std::int16_t cur)
                {
                    if (is_fake)return 0;

                    std::uint16_t statusword = this->statusWord();
                    int motorState = (statusword & 0x000F);

                    std::uint8_t mode_Read = this->operationMode();

                    if (motorState != 0x0007 || mode_Read != CURRENT) //need running and cur mode
                    {
                        return -1;
                    }
                    else
                    {
                        pdrv->writePdo(io_mapping_->targetTorque_index, 
                                io_mapping_->targetTorque_subindex,
                                cur);
                        return 0;
                    }
                }

                std::int32_t pos()
                {
                    std::int32_t pos;
                    pdrv->readPdo(
                            io_mapping_->positionActualValue_index, 
                            io_mapping_->positionActualValue_subindex, 
                            pos); 					
                    return pos + pos_offset_;
                };

                std::int32_t vel() 
                {
                    std::int32_t vel;
                    pdrv->readPdo(
                            io_mapping_->velocityActualValue_index, 
                            io_mapping_->velocityActualValue_subindex,
                            vel); 					
                    return vel;
                };

                std::int32_t cur()
                {
                    std::int16_t cur;
                    pdrv->readPdo(
                            io_mapping_->torqueActualValue_index,
                            io_mapping_->torqueActualValue_subindex,
                            cur);
                    return cur;
                };

                std::uint8_t operationMode()
                {
                    std::uint8_t om;
                    pdrv->readPdo(
                            io_mapping_->modeOfOperationDisplay_index, 
                            io_mapping_->modeOfOperationDisplay_subindex, 
                            om);
                    return om;
                };

                std::uint16_t statusWord()
                {
                    std::uint16_t sw;
                    pdrv->readPdo(
                            io_mapping_->statusWord_index, 
                            io_mapping_->statusWord_subindex,
                            sw);
                    return sw;
                };

                void writeHomeCountToDevice(std::int32_t home_count){
                    pdrv->configSdo(
                            io_mapping_->home_count_sdo_index, 
                            static_cast<std::int32_t>(home_count));
                };

                std::int32_t input2count_;
                std::int32_t home_count_;
                std::int32_t max_pos_count_;
                std::int32_t min_pos_count_;
                std::int32_t max_vel_count_;
                std::int32_t abs_id_;
                std::int32_t phy_id_;
                std::string type_;
                std::unique_ptr<MappingDefinition> io_mapping_;

                EthercatMotion *pFather;
                EthercatDriver *pdrv;

                std::int32_t pos_offset_{0};

                bool is_fake{ true };
                bool is_waiting_mode{ false };

                int enable_period{ 0 };
                int home_period { 0 };
                int waiting_count { 0 };

                std::uint8_t running_mode{ POSITION };
        };

///////////////////////////////////////////////////////////

        EthercatMotion::~EthercatMotion() {}

        EthercatMotion::EthercatMotion(
                const aris::core::XmlElement &xml_ele, 
                aris::control::EthercatDriver &physical_driver)
            :imp_(new EthercatMotion::Imp(this, &physical_driver))
        {
            imp_->SetIOMapping(xml_ele);

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

            imp_->writeHomeCountToDevice(imp_->home_count_);
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
            data.status_word  = imp_->statusWord();
        }

        auto EthercatMotion::hasFault()->bool
        {
            std::uint16_t statusword = imp_->statusWord();
            int motorState = (statusword & 0x000F);
            return (motorState != 0x0003 && motorState != 0x0007 && motorState != 0x0001 && motorState != 0x0000) ? true : false;
        }

        auto EthercatMotion::absID()->std::int32_t { return imp_->abs_id_; };

        auto EthercatMotion::phyID()->std::int32_t { return imp_->phy_id_; };

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

        auto EthercatMotion::printStatus() const -> void
        {
            rt_printf("Mot PhyID %d: sw %5d om %2d pos %7d vel %7d cur %7d\n", 
                    imp_->phy_id_, 
                    imp_->statusWord(),
                    imp_->operationMode(),
                    imp_->pos(),
                    imp_->vel(),
                    imp_->cur()
                    );
        }

        auto EthercatForceSensor::readData(Data &data)->void
        {
            std::int32_t value;

            this->readPdo(0, 0, value);
            data.Fx = static_cast<double>(value) * force_ratio_;

            this->readPdo(0, 1, value);
            data.Fy = static_cast<double>(value) * force_ratio_;

            this->readPdo(0, 2, value);
            data.Fz = static_cast<double>(value) * force_ratio_;

            this->readPdo(0, 3, value);
            data.Mx = static_cast<double>(value) * torque_ratio_;

            this->readPdo(0, 4, value);
            data.My = static_cast<double>(value) * torque_ratio_;

            this->readPdo(0, 5, value);
            data.Mz = static_cast<double>(value) * torque_ratio_;
        }
        
		auto EthercatForceSensorRuiCongCombo::readData(RuiCongComboData &data)->void
		{
			
			int32_t value=0;
			float* pdo_result=NULL;
			for( std::size_t i = 0; i < data.force.size(); i++)
			{
				// read 6 element in one loop
				this->readPdo(i, 0, value);
				pdo_result = (float*)&value;
				raw_data_.force.at(i).Fx = *pdo_result * force_ratio_;
				
				this->readPdo(i, 1, value);
				pdo_result = (float*)&value;
				raw_data_.force.at(i).Fy = *pdo_result * force_ratio_;

				this->readPdo(i, 2, value);
				pdo_result = (float*)&value;
				raw_data_.force.at(i).Fz = *pdo_result * force_ratio_;

				this->readPdo(i, 3, value);
				pdo_result = (float*)&value;
				raw_data_.force.at(i).Mx = *pdo_result * torque_ratio_;

				this->readPdo(i, 4, value);
				pdo_result = (float*)&value;
				raw_data_.force.at(i).My = *pdo_result * torque_ratio_;

				this->readPdo(i, 5, value);
				pdo_result = (float*)&value;
				raw_data_.force.at(i).Mz = *pdo_result * torque_ratio_;

				std::uint8_t zero_pdo_value;

				if (this->zeroing_count_left.at(i) == 1)
				{
					zero_pdo_value = 1;
					this->writePdo(CHANNEL_COUNTS+1, i, zero_pdo_value);
					this->zeroing_count_left.at(i)--;
				}
				else if (this->zeroing_count_left.at(i) == 0)
				{
					zero_pdo_value = 0;
					this->writePdo(CHANNEL_COUNTS+1, i, zero_pdo_value);
					this->zeroing_count_left.at(i)--;
					rt_printf("zeroing sensor %d\n",i);
				}
				else
				{
					zero_pdo_value = 0;
					this->writePdo(CHANNEL_COUNTS+1, i, zero_pdo_value);
				}

				data.force.at(i).Fx = raw_data_.force.at(i).Fx;
				data.force.at(i).Fy = raw_data_.force.at(i).Fy;
				data.force.at(i).Fz = raw_data_.force.at(i).Fz;
				data.force.at(i).Mx = raw_data_.force.at(i).Mx;
				data.force.at(i).My = raw_data_.force.at(i).My;
				data.force.at(i).Mz = raw_data_.force.at(i).Mz;

			}
		}

		auto EthercatForceSensorRuiCongCombo::setRatio(double f_ratio, double t_ratio)->void
		{
			this->force_ratio_ = f_ratio;
			this->torque_ratio_ = t_ratio;
		}

		auto EthercatForceSensorRuiCongCombo::requireZeroing(int sensor_id)->void
		{
            if (sensor_id >= CHANNEL_COUNTS)  // check if this device have such sensor channel
                return;
			
			if (this->zeroing_count_left.at(sensor_id) < 0)//this means it is not in a zeroing process
			{
				this->zeroing_count_left.at(sensor_id) = this->ZEROING_COUNT;
				for (int i = 0; i < 6; i++)
				{
					this->sum_data_.force.at(i).Fx = 0.0;
					this->sum_data_.force.at(i).Fy = 0.0;
					this->sum_data_.force.at(i).Fz = 0.0;
					this->sum_data_.force.at(i).Mx = 0.0;
					this->sum_data_.force.at(i).My = 0.0;
					this->sum_data_.force.at(i).Mz = 0.0;
				}
			}
			return ;

 		}

        auto EthercatIMU::readData(Data &data)->void
        {
            std::int32_t value;
            std::int16_t highbyte;
            std::int16_t lowbyte;
            for ( int i = 0; i < 3; i++)
            {
                this->readPdo(0, i, value);
                highbyte = (value & 0xFFFF0000) >> 16;
                lowbyte = (value & 0x0000FFFF) >> 8;

                data.gyro[i] = highbyte * gyro_h_resolution + lowbyte * gyro_l_resolution;
                // convert from deg/s to rad/s
                data.gyro[i] *= 3.14159265358979 / 180.0;

                this->readPdo(1, i, value);
                highbyte = (value & 0xFFFF0000) >> 16;
                lowbyte = (value & 0x0000FFFF) >> 8;

                data.accel[i] = highbyte * accel_h_resolution + lowbyte * accel_l_resolution;
                // convert from milli-G to m/s^2
                data.accel[i] *= 9.806 / 1000.0;

            }
            
            madgwick_filter.updateIMU(
                    data.gyro[0], data.gyro[1], data.gyro[2], 
                    data.accel[0], data.accel[1], data.accel[2]);

            data.euler[0] = madgwick_filter.getRollRadians();
            data.euler[1] = madgwick_filter.getPitchRadians();
            data.euler[2] = madgwick_filter.getYawRadians();
        }
 		
        struct EthercatController::Imp
        {
            // The mapping between the physical drivers and the logical axes (abstract axes)
            std::vector<int> map_phy2abs_, map_abs2phy_;

            std::function<int(Data&)> strategy_;
            Pipe<aris::core::Msg> msg_pipe_;
            std::atomic_bool is_stopping_;

            std::vector<EthercatDriver *> driver_vec_; // physical driver devices
            std::vector<std::unique_ptr<EthercatMotion> > motion_vec_; // logical axes
            std::vector<EthercatMotion::RawData> motion_rawdata_, last_motion_rawdata_;

            std::vector<EthercatForceSensor *> force_sensor_vec_;
            std::vector<EthercatForceSensor::Data> force_sensor_data_;
            
			std::vector<EthercatForceSensorRuiCongCombo *> force_sensor_rcc_vec_;
			std::vector<EthercatForceSensorRuiCongCombo::RuiCongComboData> force_sensor_rcc_data_;

            std::vector<EthercatIMU *> imu_vec_;
            std::vector<EthercatIMU::Data> imu_data_;

            std::unique_ptr<Pipe<std::vector<EthercatMotion::RawData> > > record_pipe_;
            std::thread record_thread_;

            std::int32_t control_count_;

            const std::int32_t record_interval_ = 2;
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
            imp_->driver_vec_.clear();
            imp_->motion_vec_.clear();
            imp_->force_sensor_vec_.clear();
 			imp_->force_sensor_rcc_vec_.clear();
 			imp_->imu_vec_.clear();

            auto slave_xml = xml_ele.FirstChildElement("Slave");
            printf("Adding slaves\n");
            for (auto sla = slave_xml->FirstChildElement(); sla; sla = sla->NextSiblingElement())
            {
                std::string type{ sla->Attribute("type") };
                if (type == "ElmoSoloWhistle" || type == "Faulhaber" || type == "CopleyAE2")
                {
                    imp_->driver_vec_.push_back(addSlave<EthercatDriver>(std::ref(*slaveTypeMap.at(type))));
                    printf("Adding driver %s\n", type.c_str());
                }
                else if (type == "AtiForceSensor")
                {
                    imp_->force_sensor_vec_.push_back(addSlave<EthercatForceSensor>(std::ref(*slaveTypeMap.at(type))));
                }
				else if (type == "RuiCongCombo")
				{
					imp_->force_sensor_rcc_vec_.push_back(addSlave<EthercatForceSensorRuiCongCombo>(std::ref(*slaveTypeMap.at(type))));
                    printf("Adding FSR %s\n", type.c_str());
				}
                else if (type == "MeHeavyEIMU")
                {
                    imp_->imu_vec_.push_back(addSlave<EthercatIMU>(std::ref(*slaveTypeMap.at(type))));
                    printf("Adding IMU %s\n", type.c_str());
                }
                else
                {
                    throw std::runtime_error(std::string("unknown slave type of \"") + type + "\"");
                }
            }

            // Load all logic axes
            auto motion_xml = xml_ele.FirstChildElement("LogicMotionAxis");

            printf("Adding LogicMotionAxis\n");
            for (auto axis = motion_xml->FirstChildElement(); axis; axis = axis->NextSiblingElement())
            {
                std::string type{ axis->Attribute("type") };
                if (type == "ElmoSoloWhistle" || type == "Faulhaber" || type == "CopleyAE2_A" || type == "CopleyAE2_B")
                {
                    int driver_id = 0;
                    if (axis->QueryIntAttribute("driver_id", &driver_id) != tinyxml2::XML_NO_ERROR)
                    {
                        throw std::runtime_error("failed to find motion attribute \"driver_id\"");
                    }

                    imp_->motion_vec_.push_back(
                            std::unique_ptr<EthercatMotion>(new EthercatMotion(
                                    std::ref(*axis), *(imp_->driver_vec_[driver_id])
                                    )
                                )
                            );
                }
                else
                {
                    throw std::runtime_error(std::string("unknown axis type of \"") + type + "\"");
                }
            }

            /*update map*/
            imp_->map_phy2abs_.resize(imp_->motion_vec_.size());
            imp_->map_abs2phy_.resize(imp_->motion_vec_.size());

            for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
            {
                imp_->map_phy2abs_[i] = imp_->motion_vec_[i]->absID();
                motionAtPhy(i).imp_->phy_id_ = i;
            }

            for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
            {
                imp_->map_abs2phy_[i] = std::find(imp_->map_phy2abs_.begin(), imp_->map_phy2abs_.end(), i) - imp_->map_phy2abs_.begin();
            }

            /*resize other var*/
            imp_->motion_rawdata_.resize(imp_->motion_vec_.size());
            imp_->last_motion_rawdata_.resize(imp_->motion_vec_.size());
            imp_->force_sensor_data_.resize(imp_->force_sensor_vec_.size());
			imp_->force_sensor_rcc_data_.resize(imp_->force_sensor_rcc_vec_.size());
			imp_->imu_data_.resize(imp_->imu_vec_.size());

            imp_->record_pipe_.reset(new Pipe<std::vector<EthercatMotion::RawData> >(true, imp_->motion_vec_.size()));
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
            imp_->control_count_ = 0;

            /*begin thread which will save data*/
            if(!imp_->record_thread_.joinable())
                imp_->record_thread_ = std::thread([this]()
                        {
                            static std::fstream file;
                            std::string name = aris::core::logFileName();
                            name.replace(name.rfind("log.txt"), std::strlen("data.txt"), "data.txt");
                            file.open(name.c_str(), std::ios::out | std::ios::trunc);

                            std::vector<EthercatMotion::RawData> data;
                            data.resize(imp_->motion_vec_.size());

                            long long count = -1;
                            while (!imp_->is_stopping_)
                            {
                                imp_->record_pipe_->recvInNrt(data);

                                count += imp_->record_interval_;

                                file << count << " ";

                                for (auto &d : data)
                                {
                                    file << d.status_word  << " ";
                                    file << d.target_pos   << " ";
                                    file << d.feedback_pos << " ";
                                    file << d.feedback_vel << " ";
                                    file << d.feedback_cur << " ";
                                }
                                file << std::endl;
                            }

                            file.close();
                            std::cout << "record thread finished"<< std::endl;
                        });

            this->EthercatMaster::start();
        }

        auto EthercatController::stop()->void
        {
            this->EthercatMaster::stop();
            rt_printf("Ethercat Master stopped\n");
            imp_->record_thread_.detach();
        }

        auto EthercatController::motionNum()->std::size_t { return imp_->motion_vec_.size(); };
        auto EthercatController::motionAtAbs(int i)->EthercatMotion & { return *imp_->motion_vec_.at(imp_->map_abs2phy_[i]); };
        auto EthercatController::motionAtPhy(int i)->EthercatMotion & { return *imp_->motion_vec_.at(i); };
        auto EthercatController::forceSensorNum()->std::size_t { return imp_->force_sensor_vec_.size(); };
        auto EthercatController::forceSensorAt(int i)->EthercatForceSensor & { return *imp_->force_sensor_vec_.at(i); };
		auto EthercatController::ruicongComboNum()->std::size_t { return imp_->force_sensor_rcc_vec_.size(); };
		auto EthercatController::ruicongComboAt(int i)->EthercatForceSensorRuiCongCombo & { return *imp_->force_sensor_rcc_vec_.at(i); };
        auto EthercatController::imuNum()->std::size_t { return imp_->imu_vec_.size(); };
        auto EthercatController::imuAt(int i)->EthercatIMU & { return *imp_->imu_vec_.at(i); };

        auto EthercatController::msgPipe()->Pipe<aris::core::Msg>& { return imp_->msg_pipe_; };

        auto EthercatController::controlStrategy()->void
        {
            /*构造传入strategy的参数*/
			Data data{ 
                &imp_->last_motion_rawdata_, 
                &imp_->motion_rawdata_, 
                &imp_->force_sensor_data_, 
                &imp_->force_sensor_rcc_data_, 
                &imp_->imu_data_,
                nullptr, 
                nullptr };

            /*收取消息*/
            if (this->msgPipe().recvInRT(aris::core::MsgRT::instance[0]) > 0)
            {
                data.msg_recv = &aris::core::MsgRT::instance[0];
            };

            /*读取反馈*/
			if (imp_->motion_vec_.size() > 0)
            {
				for (std::size_t i = 0; i < imp_->motion_vec_.size(); ++i)
				{
					motionAtAbs(i).readFeedback(imp_->motion_rawdata_[i]);
				}
            }
            
			if (imp_->force_sensor_vec_.size() > 0)
 			{
				for (std::size_t i = 0; i < imp_->force_sensor_vec_.size(); ++i)
				{
					imp_->force_sensor_vec_.at(i)->readData(imp_->force_sensor_data_[i]);
				}
 			}
			if (imp_->force_sensor_rcc_vec_.size() > 0)
			{
				for (std::size_t i = 0; i < imp_->force_sensor_rcc_vec_.size(); ++i)
				{
					imp_->force_sensor_rcc_vec_.at(i)->readData(imp_->force_sensor_rcc_data_[i]);
				}
			}
			if (imp_->imu_vec_.size() > 0)
 			{
				for (std::size_t i = 0; i < imp_->imu_vec_.size(); ++i)
				{
					imp_->imu_vec_.at(i)->readData(imp_->imu_data_[i]);
				}
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

			if (imp_->force_sensor_rcc_vec_.size() > 0)
			{
                for (std::size_t i = 0; i < imp_->force_sensor_rcc_data_.at(0).force.size(); i++)
                {
                    if (imp_->force_sensor_rcc_data_.at(0).isZeroingRequested.at(i))
                    {
                        imp_->force_sensor_rcc_vec_.at(0)->requireZeroing(i);
                        imp_->force_sensor_rcc_data_.at(0).isZeroingRequested.at(i) = false;
                    }
                }
            }

            /*发送数据到记录的线程*/
            if (imp_->control_count_ % imp_->record_interval_)
            {
                imp_->record_pipe_->sendToNrt(imp_->motion_rawdata_);
            }

            /*向外发送消息*/
            if (data.msg_send)
            {
                this->msgPipe().sendToNrt(*data.msg_send);
            }

            if (imp_->control_count_ % 100 == 0)
            {
            //    motionAtPhy(0).printStatus();
            //    rt_printf("Current motor cmd: %d\n", imp_->motion_rawdata_[0].cmd);
                if (imp_->imu_vec_.size() > 0){
                    rt_printf("%f\t%f\t%f\t%f\t%f\t%f\n",
                            imp_->imu_data_[0].accel[0],
                            imp_->imu_data_[0].accel[1],
                            imp_->imu_data_[0].accel[2],
                            imp_->imu_data_[0].euler[0]/3.14159265358979*180,
                            imp_->imu_data_[0].euler[1]/3.14159265358979*180,
                            imp_->imu_data_[0].euler[2]/3.14159265358979*180);
                }

                if (imp_->force_sensor_rcc_vec_.size() > 0)
                {
                    rt_printf("%f\t%f\t%f\t%f\t%f\t%f\n",
                            imp_->force_sensor_rcc_data_[0].force[0].Fx,
                            imp_->force_sensor_rcc_data_[0].force[0].Fy,
                            imp_->force_sensor_rcc_data_[0].force[0].Fz,
                            imp_->force_sensor_rcc_data_[0].force[0].Mx,
                            imp_->force_sensor_rcc_data_[0].force[0].My,
                            imp_->force_sensor_rcc_data_[0].force[0].Mz);
                }
            }

            imp_->control_count_ ++;
        }
    }
}
