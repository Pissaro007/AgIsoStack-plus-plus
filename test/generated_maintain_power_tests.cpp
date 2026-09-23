#include <gtest/gtest.h>

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_maintain_power_interface.hpp"
#include "isobus/utility/system_timing.hpp"

#include <memory>
#include <vector>

namespace isobus
{
	// Sous-classe locale de test permettant d'exposer les membres protégés de MaintainPowerInterface
	class TestableMaintainPowerInterface : public MaintainPowerInterface
	{
	public:
		using MaintainPowerInterface::MaintainPowerInterface;
		using MaintainPowerInterface::send_maintain_power;
	};

	// Fixture de test pour MaintainPowerInterface
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

	// 1. État initial et accesseurs de MaintainPowerData
	TEST_F(MaintainPowerInterfaceTest, MaintainPowerDataInitialState)
	{
		MaintainPowerInterface::MaintainPowerData data(internalCF1);

		EXPECT_EQ(data.get_sender_control_function(), internalCF1);
		EXPECT_EQ(data.get_implement_in_work_state(), MaintainPowerInterface::MaintainPowerData::ImplementInWorkState::NotAvailable);
		EXPECT_EQ(data.get_implement_ready_to_work_state(), MaintainPowerInterface::MaintainPowerData::ImplementReadyToWorkState::NotAvailable);
		EXPECT_EQ(data.get_implement_park_state(), MaintainPowerInterface::MaintainPowerData::ImplementParkState::NotAvailable);
		EXPECT_EQ(data.get_implement_transport_state(), MaintainPowerInterface::MaintainPowerData::ImplementTransportState::NotAvailable);
		EXPECT_EQ(data.get_maintain_actuator_power(), MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::DontCare);
		EXPECT_EQ(data.get_maintain_ecu_power(), MaintainPowerInterface::MaintainPowerData::MaintainECUPower::DontCare);
	}

	// 2. Valeurs de retour des setters de MaintainPowerData
	TEST_F(MaintainPowerInterfaceTest, MaintainPowerDataSettersReturnValues)
	{
		MaintainPowerInterface::MaintainPowerData data(internalCF1);

		// Premier changement : doit retourner true
		EXPECT_TRUE(data.set_implement_in_work_state(MaintainPowerInterface::MaintainPowerData::ImplementInWorkState::ImplementInWorkState));
		EXPECT_EQ(data.get_implement_in_work_state(), MaintainPowerInterface::MaintainPowerData::ImplementInWorkState::ImplementInWorkState);

		// Même valeur : doit retourner false
		EXPECT_FALSE(data.set_implement_in_work_state(MaintainPowerInterface::MaintainPowerData::ImplementInWorkState::ImplementInWorkState));

		EXPECT_TRUE(data.set_implement_ready_to_work_state(MaintainPowerInterface::MaintainPowerData::ImplementReadyToWorkState::ImplementReadyForFieldWork));
		EXPECT_FALSE(data.set_implement_ready_to_work_state(MaintainPowerInterface::MaintainPowerData::ImplementReadyToWorkState::ImplementReadyForFieldWork));

		EXPECT_TRUE(data.set_implement_park_state(MaintainPowerInterface::MaintainPowerData::ImplementParkState::ImplementMayBeDisconnected));
		EXPECT_FALSE(data.set_implement_park_state(MaintainPowerInterface::MaintainPowerData::ImplementParkState::ImplementMayBeDisconnected));

		EXPECT_TRUE(data.set_implement_transport_state(MaintainPowerInterface::MaintainPowerData::ImplementTransportState::ImplementMayBeTransported));
		EXPECT_FALSE(data.set_implement_transport_state(MaintainPowerInterface::MaintainPowerData::ImplementTransportState::ImplementMayBeTransported));

		EXPECT_TRUE(data.set_maintain_actuator_power(MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::RequirementFor2SecondsMoreForPWR));
		EXPECT_FALSE(data.set_maintain_actuator_power(MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::RequirementFor2SecondsMoreForPWR));

		EXPECT_TRUE(data.set_maintain_ecu_power(MaintainPowerInterface::MaintainPowerData::MaintainECUPower::RequirementFor2SecondsMoreForECU_PWR));
		EXPECT_FALSE(data.set_maintain_ecu_power(MaintainPowerInterface::MaintainPowerData::MaintainECUPower::RequirementFor2SecondsMoreForECU_PWR));
	}

	// 3. Initialisation et idempotence de MaintainPowerInterface
	TEST_F(MaintainPowerInterfaceTest, InitializationAndIdempotency)
	{
		MaintainPowerInterface interface(internalCF1);

		EXPECT_FALSE(interface.get_initialized());

		interface.initialize();
		EXPECT_TRUE(interface.get_initialized());

		// L'initialisation répétée doit être idempotente sans altérer l'état
		interface.initialize();
		EXPECT_TRUE(interface.get_initialized());
	}

	// 4. Configuration du temps de maintien d'alimentation
	TEST_F(MaintainPowerInterfaceTest, MaintainPowerDurationConfiguration)
	{
		MaintainPowerInterface interface(internalCF1);

		EXPECT_EQ(interface.get_maintain_power_time(), 0U); // Valeur par défaut : 0 ms

		interface.set_maintain_power_time(5000U);
		EXPECT_EQ(interface.get_maintain_power_time(), 5000U);
	}

	// 5. Gestion sûre des accès hors limites
	TEST_F(MaintainPowerInterfaceTest, OutOfBoundsIndexHandling)
	{
		MaintainPowerInterface interface(internalCF1);
		interface.initialize();

		EXPECT_EQ(interface.get_number_received_maintain_power_sources(), 0U);
		EXPECT_EQ(interface.get_received_maintain_power(0U), nullptr);
		EXPECT_EQ(interface.get_received_maintain_power(999U), nullptr);
	}

	// 6. Enregistrement des callbacks via Event Publisher
	TEST_F(MaintainPowerInterfaceTest, EventPublisherRegistration)
	{
		MaintainPowerInterface interface(internalCF1);
		interface.initialize();

		bool callbackTriggered = false;

		interface.get_maintain_power_data_event_publisher().add_listener([&](const std::shared_ptr<MaintainPowerInterface::MaintainPowerData> &, bool) {
			callbackTriggered = true;
		});

		EXPECT_FALSE(callbackTriggered);
	}

	// 7. Encodage et transmission d'un message Maintain Power via la méthode protégée
	TEST_F(MaintainPowerInterfaceTest, SendMaintainPowerMessage)
	{
		TestableMaintainPowerInterface interface(internalCF1);
		interface.initialize();

		interface.maintainPowerTransmitData.set_maintain_ecu_power(MaintainPowerInterface::MaintainPowerData::MaintainECUPower::RequirementFor2SecondsMoreForECU_PWR);
		interface.maintainPowerTransmitData.set_maintain_actuator_power(MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::RequirementFor2SecondsMoreForPWR);

		EXPECT_TRUE(interface.send_maintain_power());
	}

	// 8. Nettoyage et mise à jour de l'interface
	TEST_F(MaintainPowerInterfaceTest, InterfaceUpdateCycle)
	{
		MaintainPowerInterface interface(internalCF1);
		interface.initialize();

		interface.update();

		EXPECT_EQ(interface.get_number_received_maintain_power_sources(), 0U);
	}
} // namespace isobus
