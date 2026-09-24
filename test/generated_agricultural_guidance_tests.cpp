#include <gtest/gtest.h>

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_identifier.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_guidance_interface.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/virtual_can_plugin.hpp"
#include "helpers/control_function_helpers.hpp"
#include "helpers/test_fixture.hpp"

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

	class AgriculturalGuidanceInterfaceTest : public AgIsoStackTestFixture
	{
	protected:
		void SetUp() override
		{
			AgIsoStackTestFixture::SetUp();

			testPlugin.open();
			CANHardwareInterface::set_number_of_can_channels(1);
			CANHardwareInterface::assign_can_channel_frame_handler(0, std::make_shared<VirtualCANPlugin>());
			CANHardwareInterface::start(false);

			internalSender = test_helpers::claim_internal_control_function(0x44, 0, time_source);

			NAME nameDest;
			nameDest.set_identity_number(2);
			NAME nameRx;
			nameRx.set_identity_number(3);
			externalDest = std::make_shared<ControlFunction>(nameDest, 0x20, 0);
			externalRxSource = std::make_shared<ControlFunction>(nameRx, 0x30, 0);

			CANMessageFrame frame = {};
			while (!testPlugin.get_queue_empty())
			{
				testPlugin.read_frame(frame);
			}
		}

		void TearDown() override
		{
			CANNetworkManager::CANNetwork.update();
			CANNetworkManager::CANNetwork.deactivate_control_function(internalSender);
			CANHardwareInterface::stop();
			testPlugin.close();
			AgIsoStackTestFixture::TearDown();
		}

		VirtualCANPlugin testPlugin;
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

		// Data byte 0 = 0x00, byte 1 = 0x7D -> raw uint16 = 0x7D00 = 32000
		// Decoding formula: (raw * resolution) - offset = (32000 * 0.25) - 8032 = 8000 - 8032 = -32.0 km-1
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
		EXPECT_FLOAT_EQ(-32.0f, eventCommand->get_curvature());
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

		// Data byte 0 = 0x04, byte 1 = 0x7D -> raw uint16 = 0x7D04 = 32004
		// Decoding formula: (raw * resolution) - offset = (32004 * 0.25) - 8032 = 8001 - 8032 = -31.0 km-1
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
		EXPECT_FLOAT_EQ(-31.0f, eventInfo->get_estimated_curvature());
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
		TestableAgriculturalGuidanceInterface interface(internalSender, externalDest, true, false);
		CANMessageFrame transmittedFrame = {};

		// 1. Normal value: 0 km-1
		interface.guidanceSystemCommandTransmitData.set_curvature(0.0f);
		interface.guidanceSystemCommandTransmitData.set_status(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer);
		ASSERT_TRUE(interface.send_guidance_system_command());
		time_source.update_for_ms(5);
		ASSERT_TRUE(testPlugin.read_frame(transmittedFrame));
		ASSERT_EQ(8, transmittedFrame.dataLength);
		EXPECT_EQ(32128, static_cast<std::uint16_t>(transmittedFrame.data[0]) | (static_cast<std::uint16_t>(transmittedFrame.data[1]) << 8));
		EXPECT_EQ(0xFD, transmittedFrame.data[2]);

		// 2. Maximum clamping
		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(9000.0f);
		interface.guidanceSystemCommandTransmitData.set_curvature(9000.0f);
		ASSERT_TRUE(interface.send_guidance_system_command());
		time_source.update_for_ms(5);
		ASSERT_TRUE(testPlugin.read_frame(transmittedFrame));
		ASSERT_EQ(8, transmittedFrame.dataLength);
		EXPECT_EQ(32127 + 32128, static_cast<std::uint16_t>(transmittedFrame.data[0]) | (static_cast<std::uint16_t>(transmittedFrame.data[1]) << 8));

		// Reset estimated curvature before testing minimum clamping.
		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(0.0f);

		// 3. Minimum clamping
		interface.guidanceSystemCommandTransmitData.set_curvature(-9000.0f);
		ASSERT_TRUE(interface.send_guidance_system_command());
		time_source.update_for_ms(5);
		ASSERT_TRUE(testPlugin.read_frame(transmittedFrame));
		ASSERT_EQ(8, transmittedFrame.dataLength);
		EXPECT_EQ(0, static_cast<std::uint16_t>(transmittedFrame.data[0]) | (static_cast<std::uint16_t>(transmittedFrame.data[1]) << 8));
	}
	TEST_F(AgriculturalGuidanceInterfaceTest, SendGuidanceMachineInfoEncodingAndClamping)
	{
		TestableAgriculturalGuidanceInterface interface(internalSender, externalDest, false, true);
		CANMessageFrame transmittedFrame = {};

		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(1.0f);
		interface.guidanceMachineInfoTransmitData.set_mechanical_system_lockout_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active);
		interface.guidanceMachineInfoTransmitData.set_guidance_steering_system_readiness_state(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive);
		interface.guidanceMachineInfoTransmitData.set_guidance_steering_input_position_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::DisabledOffPassive);
		interface.guidanceMachineInfoTransmitData.set_request_reset_command_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetNotRequired);
		interface.guidanceMachineInfoTransmitData.set_guidance_limit_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedHigh);
		interface.guidanceMachineInfoTransmitData.set_guidance_system_command_exit_reason_code(static_cast<std::uint8_t>(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceSystemCommandExitReasonCode::OperatorOverrideOfFunction));
		interface.guidanceMachineInfoTransmitData.set_guidance_system_remote_engage_switch_status(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive);

		ASSERT_TRUE(interface.send_guidance_machine_info());
		time_source.update_for_ms(5);
		ASSERT_TRUE(testPlugin.read_frame(transmittedFrame));
		ASSERT_EQ(8, transmittedFrame.dataLength);
		EXPECT_EQ(32132, static_cast<std::uint16_t>(transmittedFrame.data[0]) | (static_cast<std::uint16_t>(transmittedFrame.data[1]) << 8));
		EXPECT_EQ(0x05, transmittedFrame.data[2]);
		EXPECT_EQ(0x40, transmittedFrame.data[3]);
		EXPECT_EQ(0x43, transmittedFrame.data[4]);

		// Minimum clamping
		interface.guidanceMachineInfoTransmitData.set_estimated_curvature(-9000.0f);
		ASSERT_TRUE(interface.send_guidance_machine_info());
		time_source.update_for_ms(5);
		ASSERT_TRUE(testPlugin.read_frame(transmittedFrame));
		ASSERT_EQ(8, transmittedFrame.dataLength);
		EXPECT_EQ(0, static_cast<std::uint16_t>(transmittedFrame.data[0]) | (static_cast<std::uint16_t>(transmittedFrame.data[1]) << 8));
	}
	TEST_F(AgriculturalGuidanceInterfaceTest, SendWithoutSenderReturnsFalse)
	{
		TestableAgriculturalGuidanceInterface interface(nullptr, nullptr, false, false);
		EXPECT_FALSE(interface.send_guidance_system_command());
		EXPECT_FALSE(interface.send_guidance_machine_info());
	}

	TEST_F(AgriculturalGuidanceInterfaceTest, ProcessFlagsCallback)
	{
		TestableAgriculturalGuidanceInterface interface(internalSender, externalDest, true, true);
		CANMessageFrame transmittedFrame = {};

		TestableAgriculturalGuidanceInterface::process_flags(static_cast<std::uint32_t>(TestableAgriculturalGuidanceInterface::TransmitFlags::SendGuidanceSystemCommand), &interface);
		time_source.update_for_ms(5);
		ASSERT_TRUE(testPlugin.read_frame(transmittedFrame));
		EXPECT_EQ(8, transmittedFrame.dataLength);
		CANIdentifier systemCommandIdentifier(transmittedFrame.identifier);
		EXPECT_EQ(static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand), systemCommandIdentifier.get_parameter_group_number());

		TestableAgriculturalGuidanceInterface::process_flags(static_cast<std::uint32_t>(TestableAgriculturalGuidanceInterface::TransmitFlags::SendGuidanceMachineInfo), &interface);
		time_source.update_for_ms(5);
		ASSERT_TRUE(testPlugin.read_frame(transmittedFrame));
		EXPECT_EQ(8, transmittedFrame.dataLength);
		CANIdentifier machineInfoIdentifier(transmittedFrame.identifier);
		EXPECT_EQ(static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo), machineInfoIdentifier.get_parameter_group_number());

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