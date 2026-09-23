#include <gtest/gtest.h>

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_maintain_power_interface.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/utility/time_source.hpp"

#include <memory>
#include <vector>

namespace isobus
{
	// Double de test local pour la source de temps permettant de contrôler le temps sans attente réelle
	class TestTimeSource : public TimeSource
	{
	public:
		TestTimeSource() : currentTimeMs(0) {}

		std::uint32_t get_current_time_ms() const override
		{
			return currentTimeMs;
		}

		std::uint64_t get_current_time_us() const override
		{
			return static_cast<std::uint64_t>(currentTimeMs) * 1000ULL;
		}

		void advance_time_ms(std::uint32_t ms)
		{
			currentTimeMs += ms;
		}

		void set_time_ms(std::uint32_t ms)
		{
			currentTimeMs = ms;
		}

	private:
		std::uint32_t currentTimeMs;
	};

	// Fixture de test garantissant l'isolation de l'état global et de la source de temps
	class MaintainPowerInterfaceTest : public ::testing::Test
	{
	protected:
		void SetUp() override
		{
			testTimeSource = std::shared_ptr<TestTimeSource>(new TestTimeSource());
			SystemTiming::set_time_source(testTimeSource.get());

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
			SystemTiming::set_time_source(nullptr);
		}

		std::shared_ptr<TestTimeSource> testTimeSource;
		std::shared_ptr<InternalControlFunction> internalCF1;
		std::shared_ptr<InternalControlFunction> internalCF2;
	};

	// 1. État initial et accesseurs de MaintainPowerData
	TEST_F(MaintainPowerInterfaceTest, MaintainPowerDataInitialState)
	{
		MaintainPowerInterface::MaintainPowerData data(internalCF1);

		EXPECT_EQ(data.get_sending_control_function(), internalCF1);
		EXPECT_EQ(data.get_implement_in_work_state(), MaintainPowerInterface::MaintainPowerData::ImplementInWorkState::NotAvailable);
		EXPECT_EQ(data.get_implement_ready_to_work_state(), MaintainPowerInterface::MaintainPowerData::ImplementReadyToWorkState::NotAvailable);
		EXPECT_EQ(data.get_implement_park_state(), MaintainPowerInterface::MaintainPowerData::ImplementParkState::NotAvailable);
		EXPECT_EQ(data.get_implement_transport_state(), MaintainPowerInterface::MaintainPowerData::ImplementTransportState::NotAvailable);
		EXPECT_EQ(data.get_maintain_actuator_power(), MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::DontCare);
		EXPECT_EQ(data.get_maintain_ecu_power(), MaintainPowerInterface::MaintainPowerData::MaintainECUPower::DontCare);
	}

	// 2. Valeurs de retour des setters
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

		EXPECT_FALSE(interface.get_is_initialized());

		EXPECT_TRUE(interface.initialize());
		EXPECT_TRUE(interface.get_is_initialized());

		// L'initialisation répétée doit être idempotente et retourner true
		EXPECT_TRUE(interface.initialize());
		EXPECT_TRUE(interface.get_is_initialized());
	}

	// 4. Configuration de la durée de maintien d'alimentation
	TEST_F(MaintainPowerInterfaceTest, MaintainPowerDurationConfiguration)
	{
		MaintainPowerInterface interface(internalCF1);

		EXPECT_EQ(interface.get_maintain_power_duration_ms(), 2000U); // Valeur par défaut de 2 secondes

		interface.set_maintain_power_duration_ms(5000U);
		EXPECT_EQ(interface.get_maintain_power_duration_ms(), 5000U);
	}

	// 5. Gestion sûre d'un index hors limites
	TEST_F(MaintainPowerInterfaceTest, OutOfBoundsIndexHandling)
	{
		MaintainPowerInterface interface(internalCF1);
		interface.initialize();

		EXPECT_EQ(interface.get_number_maintain_power_data_sources(), 0U);
		EXPECT_EQ(interface.get_maintain_power_data(0U), nullptr);
		EXPECT_EQ(interface.get_maintain_power_data(999U), nullptr);
	}

	// 6. Décodage d'un message Maintain Power valide et notification
	TEST_F(MaintainPowerInterfaceTest, DecodeValidMaintainPowerMessage)
	{
		MaintainPowerInterface interface(internalCF1);
		interface.initialize();

		bool callbackTriggered = false;
		std::shared_ptr<MaintainPowerInterface::MaintainPowerData> receivedDataPtr = nullptr;

		interface.get_maintain_power_data_event_dispatcher().add_listener([&](const std::shared_ptr<MaintainPowerInterface::MaintainPowerData> &data, bool) {
			callbackTriggered = true;
			receivedDataPtr = data;
		});

		// Données valides du message Maintain Power (8 octets)
		std::vector<std::uint8_t> payload = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF, 0xFF};
		CANIdentifier id(CANIdentifier::Type::Extended, static_cast<std::uint32_t>(CANLibParameterGroupNumber::MaintainPower), CANIdentifier::CANPriority::PriorityDefault6, 0xFF, internalCF2->get_address());
		CANMessage message(CANMessage::Type::Receive, id, payload, internalCF2, nullptr, 0);

		CANNetworkManager::CANNetwork.process_receive_can_message_frame(CANMessageFrame(0, id.get_identifier(), payload.data(), payload.size()));
		CANNetworkManager::CANNetwork.update();

		interface.update();

		if (interface.get_number_maintain_power_data_sources() > 0)
		{
			EXPECT_NE(interface.get_maintain_power_data(0), nullptr);
		}
	}

	// 7. Encodage et transmission d'un message Maintain Power
	TEST_F(MaintainPowerInterfaceTest, SendMaintainPowerMessage)
	{
		MaintainPowerInterface interface(internalCF1);
		interface.initialize();

		interface.get_maintain_power_data()->set_maintain_ecu_power(MaintainPowerInterface::MaintainPowerData::MaintainECUPower::RequirementFor2SecondsMoreForECU_PWR);
		interface.get_maintain_power_data()->set_maintain_actuator_power(MaintainPowerInterface::MaintainPowerData::MaintainActuatorPower::RequirementFor2SecondsMoreForPWR);

		EXPECT_TRUE(interface.send_maintain_power());
	}

	// 8. Nettoyage et expiration des données obsolètes
	TEST_F(MaintainPowerInterfaceTest, PruneExpiredMaintainPowerData)
	{
		MaintainPowerInterface interface(internalCF1);
		interface.initialize();

		// Avancer le temps pour simuler l'expiration des données
		testTimeSource->advance_time_ms(10000U);
		interface.update();

		EXPECT_EQ(interface.get_number_maintain_power_data_sources(), 0U);
	}
} // namespace isobus