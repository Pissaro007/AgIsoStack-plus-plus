#include <gtest/gtest.h>
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_maintain_power_interface.hpp"
#include "isobus/utility/system_timing.hpp"
#include <memory>
#include <vector>
namespace isobus
{
	class TestableMaintainPowerInterface : public MaintainPowerInterface
	{
	public:
		using MaintainPowerInterface::MaintainPowerInterface;
		using MaintainPowerInterface::send_maintain_power;
	};
	class MaintainPowerInterfaceTest : public ::testing::Test
	{
	protected:
		void SetUp() override
		{
			CANNetworkManager::CANNetwork.initialize();
			NAME name1(0);
			name1.set_arbitrary_address_capable(true);
			internalCF1 = CANNetworkManager::CANNetwork.create_internal_control_function(name1, 0, 0x1C);
			NAME name2(0);
			name2.set_arbitrary_address_capable(true);
			internalCF2 = CANNetworkManager::CANNetwork.create_internal_control_function(name2, 0, 0x1D);
		}
		void TearDown() override
		{
			if (internalCF1)
			{
				CANNetworkManager::CANNetwork.deactivate_control_function(internalCF1);
			}
			if (internalCF2)
			{
				CANNetworkManager::CANNetwork.deactivate_control_function(internalCF2);
			}
		}
		std::shared_ptr<InternalControlFunction> internalCF1;
		std::shared_ptr<InternalControlFunction> internalCF2;
	};

	TEST_F(MaintainPowerInterfaceTest, MaintainPowerDurationConfiguration)
	{
		MaintainPowerInterface interface(internalCF1);
		EXPECT_EQ(interface.get_maintain_power_time(), 0U);
		interface.set_maintain_power_time(5000U);
		EXPECT_EQ(interface.get_maintain_power_time(), 5000U);
	}

	TEST_F(MaintainPowerInterfaceTest, SendMaintainPowerMessage)
	{
		TestableMaintainPowerInterface interface(internalCF1);
		interface.initialize();

		interface.maintainPowerTransmitData.set_maintain_ecu_power(
			MaintainPowerInterface::MaintainPowerData::MaintainECUPower::RequirementFor2SecondsMoreForECU_PWR);

		interface.maintainPowerTransmitData.set_maintain_actuator_power(
			MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::RequirementFor2SecondsMoreForPWR);

		EXPECT_EQ(
			interface.maintainPowerTransmitData.get_maintain_ecu_power(),
			MaintainPowerInterface::MaintainPowerData::MaintainECUPower::RequirementFor2SecondsMoreForECU_PWR);

		EXPECT_EQ(
			interface.maintainPowerTransmitData.get_maintain_actuator_power(),
			MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::RequirementFor2SecondsMoreForPWR);
	}
}
