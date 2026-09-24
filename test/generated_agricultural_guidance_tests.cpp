#include <gtest/gtest.h>

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_identifier.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_guidance_interface.hpp"

#include <cmath>
#include <memory>
#include <vector>

namespace isobus
{
	class TestableAgriculturalGuidanceInterface : public AgriculturalGuidanceInterface
	{
	public:
		using AgriculturalGuidanceInterface::AgriculturalGuidanceInterface;
		using AgriculturalGuidanceInterface::process_flags;
		using AgriculturalGuidanceInterface::process_rx_message;
		using AgriculturalGuidanceInterface::send_guidance_machine_info;
		using AgriculturalGuidanceInterface::send_guidance_system_command;
		using AgriculturalGuidanceInterface::TransmitFlags;
	};

	class AgriculturalGuidanceInterfaceTest : public ::testing::Test
	{
	protected:
		void SetUp() override
		{
			CANNetworkManager::CANNetwork.initialize();

			NAME nameSender;
			nameSender.set_arbitrary_address_capable(true);
			nameSender.set_industry_group(static_cast<std::uint8_t>(NAME::IndustryGroup::AgriculturalAndForestryEquipment));
			nameSender.set_device_class(0);
			nameSender.set_function_code(0);
			nameSender.set_identity_number(1);
			nameSender.set_manufacturer_code(1);

			NAME nameDest = nameSender;
			nameDest.set_identity_number(2);

			NAME nameRx = nameSender;
			nameRx.set_identity_number(3);

			internalSender = CANNetworkManager::CANNetwork.create_internal_control_function(nameSender, 0, 0);
			externalDest = std::make_shared<ControlFunction>(nameDest, 0x20, 0);
			externalRxSource = std::make_shared<ControlFunction>(nameRx, 0x30, 0);
		}

		std::shared_ptr<InternalControlFunction> internalSender;
		std::shared_ptr<ControlFunction> externalDest;
		std::shared_ptr<ControlFunction> externalRxSource;
	};

	TEST_F(AgriculturalGuidanceInterfaceTest, GuidanceSystemCommandSetAndGet)
	{
		AgriculturalGuidanceInterface::GuidanceSystemCommand command(externalRxSource);

		EXPECT_EQ(externalRxSource, command.get_sender_control_function());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::NotAvailable, command.get_status());
		EXPECT_FLOAT_EQ(0.0f, command.get_curvature());
		EXPECT_EQ(0u, command.get_timestamp_ms());

		EXPECT_TRUE(command.set_status(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer));
		EXPECT_FALSE(command.set_status(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer, command.get_status());

		EXPECT_TRUE(command.set_curvature(1.5f));
		EXPECT_FALSE(command.set_curvature(1.5f));
		EXPECT_FLOAT_EQ(1.5f, command.get_curvature());

		command.set_timestamp_ms(1234u);
		EXPECT_EQ(1234u, command.get_timestamp_ms());
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, GuidanceMachineInfoSetAndGet)
	{
		AgriculturalGuidanceInterface::GuidanceMachineInfo info(externalRxSource);

		EXPECT_EQ(externalRxSource, info.get_sender_control_function());
		EXPECT_FLOAT_EQ(0.0f, info.get_estimated_curvature());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::NotAvailable, info.get_mechanical_system_lockout());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::NotAvailableTakeNoAction, info.get_guidance_steering_system_readiness_state());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::NotAvailableTakeNoAction, info.get_guidance_steering_input_position_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::NotAvailable, info.get_request_reset_command_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::NotAvailable, info.get_guidance_limit_status());
		EXPECT_EQ(static_cast<std::uint8_t>(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceSystemCommandExitReasonCode::NotAvailable), info.get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::NotAvailableTakeNoAction, info.get_guidance_system_remote_engage_switch_status());
		EXPECT_EQ(0u, info.get_timestamp_ms());

		EXPECT_TRUE(info.set_estimated_curvature(-2.25f));
		EXPECT_FALSE(info.set_estimated_curvature(-2.25f));
		EXPECT_FLOAT_EQ(-2.25f, info.get_estimated_curvature());

		EXPECT_TRUE(info.set_mechanical_system_lockout_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::NotActive));
		EXPECT_FALSE(info.set_mechanical_system_lockout_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::NotActive));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::NotActive, info.get_mechanical_system_lockout());

		EXPECT_TRUE(info.set_guidance_steering_system_readiness_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_FALSE(info.set_guidance_steering_system_readiness_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive, info.get_guidance_steering_system_readiness_state());

		EXPECT_TRUE(info.set_guidance_steering_input_position_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::DisabledOffPassive));
		EXPECT_FALSE(info.set_guidance_steering_input_position_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::DisabledOffPassive));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::DisabledOffPassive, info.get_guidance_steering_input_position_status());

		EXPECT_TRUE(info.set_request_reset_command_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired));
		EXPECT_FALSE(info.set_request_reset_command_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired, info.get_request_reset_command_status());

		EXPECT_TRUE(info.set_guidance_limit_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedHigh));
		EXPECT_FALSE(info.set_guidance_limit_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedHigh));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedHigh, info.get_guidance_limit_status());

		EXPECT_TRUE(info.set_guidance_system_command_exit_reason_code(static_cast<std::uint8_t>(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceSystemCommandExitReasonCode::OperatorOverrideOfFunction)));
		EXPECT_FALSE(info.set_guidance_system_command_exit_reason_code(static_cast<std::uint8_t>(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceSystemCommandExitReasonCode::OperatorOverrideOfFunction)));
		EXPECT_EQ(static_cast<std::uint8_t>(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceSystemCommandExitReasonCode::OperatorOverrideOfFunction), info.get_guidance_system_command_exit_reason_code());

		EXPECT_TRUE(info.set_guidance_system_remote_engage_switch_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_FALSE(info.set_guidance_system_remote_engage_switch_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive, info.get_guidance_system_remote_engage_switch_status());

		info.set_timestamp_ms(9999u);
		EXPECT_EQ(9999u, info.get_timestamp_ms());
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, InitializationLifecycle)
	{
		AgriculturalGuidanceInterface interfaceNoSend(nullptr, nullptr, false, false);
		EXPECT_FALSE(interfaceNoSend.get_initialized());

		interfaceNoSend.initialize();
		EXPECT_TRUE(interfaceNoSend.get_initialized());

		interfaceNoSend.initialize();
		EXPECT_TRUE(interfaceNoSend.get_initialized());

		AgriculturalGuidanceInterface interfaceWithSend(internalSender, externalDest, true, true);
		EXPECT_FALSE(interfaceWithSend.get_initialized());
		interfaceWithSend.initialize();
		EXPECT_TRUE(interfaceWithSend.get_initialized());
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, ProcessRxSystemCommandSuccessAndEvents)
	{
		AgriculturalGuidanceInterface interface(nullptr, nullptr);
		interface.initialize();

		std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> eventCommand = nullptr;
		bool eventChanged = false;
		std::size_t callbackCount = 0;

		interface.get_guidance_system_command_event_publisher().add_listener([&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> &cmd, bool changed) {
			eventCommand = cmd;
			eventChanged = changed;
			callbackCount++;
		});

		// 0.25 km-1 per bit, -8032 km-1 offset
		// raw 32128 -> 0 km-1
		// byte 2 -> status: IntendedToSteer (1) | 0xFC = 0xFD
		std::array<std::uint8_t, 8> data = { 0x00, 0x7D, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

		CANIdentifier id(CANIdentifier::Type::Extended,
		                 static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand),
		                 CANIdentifier::CANPriority::Priority3,
		                 0xFF,
		                 externalRxSource->get_address());

		CANMessage msg(CANMessage::Type::Receive, id, data.data(), data.size(), externalRxSource, nullptr, 0);

		TestableAgriculturalGuidanceInterface::process_rx_message(msg, &interface);

		EXPECT_EQ(1u, interface.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(1u, callbackCount);
		EXPECT_TRUE(eventChanged);
		ASSERT_NE(nullptr, eventCommand);
		EXPECT_FLOAT_EQ(0.0f, eventCommand->get_curvature());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer, eventCommand->get_status());

		std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> retrieved = interface.get_received_guidance_system_command(0);
		EXPECT_EQ(eventCommand, retrieved);
		EXPECT_EQ(nullptr, interface.get_received_guidance_system_command(1));

		// Second message with identical values -> changed = false
		TestableAgriculturalGuidanceInterface::process_rx_message(msg, &interface);
		EXPECT_EQ(1u, interface.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(2u, callbackCount);
		EXPECT_FALSE(eventChanged);
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, ProcessRxMachineInfoSuccessAndEvents)
	{
		AgriculturalGuidanceInterface interface(nullptr, nullptr);
		interface.initialize();

		std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> eventInfo = nullptr;
		bool eventChanged = false;
		std::size_t callbackCount = 0;

		interface.get_guidance_machine_info_event_publisher().add_listener([&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> &info, bool changed) {
			eventInfo = info;
			eventChanged = changed;
			callbackCount++;
		});

		// raw 32132 = 0x7D04 -> (32132 * 0.25) - 8032 = +1.0 km-1
		// byte 2: lockout=Active(1), readiness=EnabledOnActive(1)<<2, inputPos=DisabledOffPassive(0)<<4, resetReq=ResetNotRequired(0)<<6 => 0x01 | 0x04 = 0x05
		// byte 3: limitStatus=LimitedHigh(2) << 5 => 0x40
		// byte 4: exitCode=OperatorOverrideOfFunction(3) & 0x3F | remoteSwitch=EnabledOnActive(1)<<6 => 0x03 | 0x40 = 0x43
		std::array<std::uint8_t, 8> data = { 0x04, 0x7D, 0x05, 0x40, 0x43, 0xFF, 0xFF, 0xFF };

		CANIdentifier id(CANIdentifier::Type::Extended,
		                 static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo),
		                 CANIdentifier::CANPriority::Priority3,
		                 0xFF,
		                 externalRxSource->get_address());

		CANMessage msg(CANMessage::Type::Receive, id, data.data(), data.size(), externalRxSource, nullptr, 0);

		TestableAgriculturalGuidanceInterface::process_rx_message(msg, &interface);

		EXPECT_EQ(1u, interface.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(1u, callbackCount);
		EXPECT_TRUE(eventChanged);
		ASSERT_NE(nullptr, eventInfo);
		EXPECT_FLOAT_EQ(1.0f, eventInfo->get_estimated_curvature());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active, eventInfo->get_mechanical_system_lockout());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive, eventInfo->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::DisabledOffPassive, eventInfo->get_guidance_steering_input_position_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetNotRequired, eventInfo->get_request_reset_command_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedHigh, eventInfo->get_guidance_limit_status());
		EXPECT_EQ(static_cast<std::uint8_t>(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceSystemCommandExitReasonCode::OperatorOverrideOfFunction), eventInfo->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive, eventInfo->get_guidance_system_remote_engage_switch_status());

		std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> retrieved = interface.get_received_guidance_machine_info(0);
		EXPECT_EQ(eventInfo, retrieved);
		EXPECT_EQ(nullptr, interface.get_received_guidance_machine_info(1));
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, ProcessRxInvalidDataLengthAndNullSender)
	{
		AgriculturalGuidanceInterface interface(nullptr, nullptr);
		interface.initialize();

		std::array<std::uint8_t, 7> shortData = { 0 };
		CANIdentifier id(CANIdentifier::Type::Extended,
		                 static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand),
		                 CANIdentifier::CANPriority::Priority3,
		                 0xFF,
		                 externalRxSource->get_address());

		CANMessage shortMsg(CANMessage::Type::Receive, id, shortData.data(), shortData.size(), externalRxSource, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_rx_message(shortMsg, &interface);
		EXPECT_EQ(0u, interface.get_number_received_guidance_system_command_sources());

		std::array<std::uint8_t, 8> validData = { 0 };
		CANMessage nullSourceMsg(CANMessage::Type::Receive, id, validData.data(), validData.size(), nullptr, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_rx_message(nullSourceMsg, &interface);
		EXPECT_EQ(0u, interface.get_number_received_guidance_system_command_sources());
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, SendGuidanceSystemCommandEncodingAndClamping)
	{
		TestableAgriculturalGuidanceInterface interface(internalSender, externalDest, false, false);

		CANMessage sentMessage(CANMessage::Type::Transmit, CANIdentifier(0), nullptr, 0, nullptr, nullptr, 0);
		bool messageTransmitted = false;

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener([&](const CANMessage &msg) {
			if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand))
			{
				sentMessage = msg;
				messageTransmitted = true;
			}
		});

		// 1. Normal value: 0 km-1
		interface.guidanceSystemCommandTransmitData.set_curvature(0.0f);
		interface.guidanceSystemCommandTransmitData.set_status(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer);

		EXPECT_TRUE(interface.send_guidance_system_command());
		EXPECT_TRUE(messageTransmitted);
		EXPECT_EQ(8u, sentMessage.get_data_length());
		EXPECT_EQ(32128, sentMessage.get_uint16_at(0));
		EXPECT_EQ(0xFD, sentMessage.get_uint8_at(2)); // IntendedToSteer (1) | 0xFC

		// 2. Max clamping
		messageTransmitted = false;
		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(9000.0f);
		interface.guidanceSystemCommandTransmitData.set_curvature(9000.0f);
		EXPECT_TRUE(interface.send_guidance_system_command());
		EXPECT_TRUE(messageTransmitted);
		EXPECT_EQ(32127 + 32128, sentMessage.get_uint16_at(0));

		// Reset estimated curvature
		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(0.0f);

		// 3. Min clamping
		messageTransmitted = false;
		interface.guidanceSystemCommandTransmitData.set_curvature(-9000.0f);
		EXPECT_TRUE(interface.send_guidance_system_command());
		EXPECT_TRUE(messageTransmitted);
		EXPECT_EQ(0, sentMessage.get_uint16_at(0));
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, SendGuidanceMachineInfoEncodingAndClamping)
	{
		TestableAgriculturalGuidanceInterface interface(internalSender, externalDest, false, false);

		CANMessage sentMessage(CANMessage::Type::Transmit, CANIdentifier(0), nullptr, 0, nullptr, nullptr, 0);
		bool messageTransmitted = false;

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener([&](const CANMessage &msg) {
			if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo))
			{
				sentMessage = msg;
				messageTransmitted = true;
			}
		});

		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(1.0f);
		interface.guidanceMachineInfoTransmitData.set_mechanical_system_lockout_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active);
		interface.guidanceMachineInfoTransmitData.set_guidance_steering_system_readiness_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive);
		interface.guidanceMachineInfoTransmitData.set_guidance_steering_input_position_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::DisabledOffPassive);
		interface.guidanceMachineInfoTransmitData.set_request_reset_command_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetNotRequired);
		interface.guidanceMachineInfoTransmitData.set_guidance_limit_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedHigh);
		interface.guidanceMachineInfoTransmitData.set_guidance_system_command_exit_reason_code(static_cast<std::uint8_t>(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceSystemCommandExitReasonCode::OperatorOverrideOfFunction));
		interface.guidanceMachineInfoTransmitData.set_guidance_system_remote_engage_switch_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive);

		EXPECT_TRUE(interface.send_guidance_machine_info());
		EXPECT_TRUE(messageTransmitted);
		EXPECT_EQ(8u, sentMessage.get_data_length());
		EXPECT_EQ(32132, sentMessage.get_uint16_at(0));
		EXPECT_EQ(0x05, sentMessage.get_uint8_at(2));
		EXPECT_EQ(0x40, sentMessage.get_uint8_at(3));
		EXPECT_EQ(0x43, sentMessage.get_uint8_at(4));

		// Min clamping
		messageTransmitted = false;
		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(-9000.0f);
		EXPECT_TRUE(interface.send_guidance_machine_info());
		EXPECT_TRUE(messageTransmitted);
		EXPECT_EQ(0, sentMessage.get_uint16_at(0));
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, SendWithoutSenderReturnsFalse)
	{
		TestableAgriculturalGuidanceInterface interface(nullptr, nullptr, false, false);
		EXPECT_FALSE(interface.send_guidance_system_command());
		EXPECT_FALSE(interface.send_guidance_machine_info());
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, ProcessFlagsCallback)
	{
		TestableAgriculturalGuidanceInterface interface(internalSender, externalDest, false, false);

		CANMessage sentMessage(CANMessage::Type::Transmit, CANIdentifier(0), nullptr, 0, nullptr, nullptr, 0);
		bool messageTransmitted = false;

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener([&](const CANMessage &msg) {
			if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand))
			{
				sentMessage = msg;
				messageTransmitted = true;
			}
		});

		TestableAgriculturalGuidanceInterface::process_flags(static_cast<std::uint32_t>(TestableAgriculturalGuidanceInterface::TransmitFlags::SendGuidanceSystemCommand), &interface);
		EXPECT_TRUE(messageTransmitted);

		messageTransmitted = false;
		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener([&](const CANMessage &msg) {
			if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo))
			{
				sentMessage = msg;
				messageTransmitted = true;
			}
		});

		TestableAgriculturalGuidanceInterface::process_flags(static_cast<std::uint32_t>(TestableAgriculturalGuidanceInterface::TransmitFlags::SendGuidanceMachineInfo), &interface);
		EXPECT_TRUE(messageTransmitted);

		TestableAgriculturalGuidanceInterface::process_flags(999, &interface);
		TestableAgriculturalGuidanceInterface::process_flags(static_cast<std::uint32_t>(TestableAgriculturalGuidanceInterface::TransmitFlags::SendGuidanceSystemCommand), nullptr);
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, UpdateWithoutInitialize)
	{
		AgriculturalGuidanceInterface interface(internalSender, externalDest, true, true);
		interface.update();
		EXPECT_FALSE(interface.get_initialized());
	}
} // namespace isobus