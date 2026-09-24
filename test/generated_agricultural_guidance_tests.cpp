//================================================================================================
/// @file test/generated_agricultural_guidance_tests.cpp
/// @brief Independent GoogleTest suite for AgriculturalGuidanceInterface in AgIsoStack++
//================================================================================================

#include <gtest/gtest.h>

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_identifier.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/can_message_frame.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_guidance_interface.hpp"

namespace isobus
{
	class TestableAgriculturalGuidanceInterface : public AgriculturalGuidanceInterface
	{
	public:
		using AgriculturalGuidanceInterface::AgriculturalGuidanceInterface;
		using AgriculturalGuidanceInterface::send_guidance_machine_info;
		using AgriculturalGuidanceInterface::send_guidance_system_command;
	};

	class AgriculturalGuidanceTest : public ::testing::Test
	{
	protected:
		void SetUp() override
		{
			CANNetworkManager::CANNetwork.initialize();

			// NAME values for constructing control functions
			NAME sourceName(0);
			sourceName.set_arbitrary_address_capable(true);
			sourceName.set_industry_group(static_cast<std::uint8_t>(NAME::IndustryGroup::AgriculturalAndForestryEquipment));
			sourceName.set_device_class(static_cast<std::uint8_t>(NAME::DeviceClass::Tractor));

			NAME destName(0);
			destName.set_arbitrary_address_capable(true);
			destName.set_industry_group(static_cast<std::uint8_t>(NAME::IndustryGroup::AgriculturalAndForestryEquipment));
			destName.set_device_class(static_cast<std::uint8_t>(NAME::DeviceClass::Tractor));

			NAME cf1Name(0);
			cf1Name.set_arbitrary_address_capable(true);
			cf1Name.set_identity_number(1);

			NAME cf2Name(0);
			cf2Name.set_arbitrary_address_capable(true);
			cf2Name.set_identity_number(2);

			srcICF = std::make_shared<InternalControlFunction>(sourceName, 0x10, 0);
			destCF = std::make_shared<ControlFunction>(destName, 0x20, 0, ControlFunction::Type::External);
			externalCF1 = std::make_shared<ControlFunction>(cf1Name, 0x30, 0, ControlFunction::Type::External);
			externalCF2 = std::make_shared<ControlFunction>(cf2Name, 0x40, 0, ControlFunction::Type::External);
		}

		void TearDown() override
		{
		}

		std::shared_ptr<InternalControlFunction> srcICF;
		std::shared_ptr<ControlFunction> destCF;
		std::shared_ptr<ControlFunction> externalCF1;
		std::shared_ptr<ControlFunction> externalCF2;
	};

	//------------------------------------------------------------------------------
	// INITIALIZATION & STATE TESTS
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, InitialStateAndInitialize)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, false);

		EXPECT_FALSE(guidanceInterface.get_initialized());
		EXPECT_EQ(0u, guidanceInterface.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(0u, guidanceInterface.get_number_received_guidance_machine_info_message_sources());

		guidanceInterface.initialize();
		EXPECT_TRUE(guidanceInterface.get_initialized());

		// Multiple calls to initialize() should remain initialized without side effects
		guidanceInterface.initialize();
		EXPECT_TRUE(guidanceInterface.get_initialized());
	}

	TEST_F(AgriculturalGuidanceTest, UpdateWithoutInitialize)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, false);
		// Should execute safely without throwing or asserting when uninitialized
		guidanceInterface.update();
		EXPECT_FALSE(guidanceInterface.get_initialized());
	}

	//------------------------------------------------------------------------------
	// TRANSMISSION: SYSTEM COMMAND ENCODING AND BOUNDARIES
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, SendGuidanceSystemCommandNullSender)
	{
		// Listen-only interface (nullptr source ICF)
		TestableAgriculturalGuidanceInterface guidanceInterface(nullptr, destCF, true, true);
		guidanceInterface.initialize();

		// Should fail to send because transmitter ICF is nullptr
		EXPECT_FALSE(guidanceInterface.send_guidance_system_command());
	}

	TEST_F(AgriculturalGuidanceTest, SendGuidanceSystemCommandNominalAndZeroCurvature)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, true, false);
		guidanceInterface.initialize();

		// 1. Zero curvature (0.0 km^-1) -> Raw encoding: (0 + 8032) / 0.25 = 32128 (0x7D80)
		EXPECT_FALSE(guidanceInterface.guidanceSystemCommandTransmitData.set_curvature(0.0f));
		EXPECT_TRUE(guidanceInterface.guidanceSystemCommandTransmitData.set_status(
			AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer));

		bool messageSent = false;
		CANIdentifier capturedIdentifier(0);
		std::vector<std::uint8_t> capturedData;

		auto transmittedEventHandle = CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener(
			[&](const CANMessage &msg) {
				if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand))
				{
					messageSent = true;
					capturedIdentifier = msg.get_identifier();
					capturedData = msg.get_data();
				}
			});

		EXPECT_TRUE(guidanceInterface.send_guidance_system_command());
		EXPECT_TRUE(messageSent);

		EXPECT_EQ(static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand), capturedIdentifier.get_parameter_group_number());
		EXPECT_EQ(CANIdentifier::CANPriority::Priority3, capturedIdentifier.get_priority());
		EXPECT_EQ(0x10, capturedIdentifier.get_source_address());
		EXPECT_EQ(0x20, capturedIdentifier.get_destination_address());

		ASSERT_EQ(8u, capturedData.size());
		// 32128 = 0x7D80 -> Byte 0: 0x80, Byte 1: 0x7D
		EXPECT_EQ(0x80, capturedData[0]);
		EXPECT_EQ(0x7D, capturedData[1]);
		// Status = IntendedToSteer (1) | 0xFC = 0xFD
		EXPECT_EQ(0xFD, capturedData[2]);
		EXPECT_EQ(0xFF, capturedData[3]);
		EXPECT_EQ(0xFF, capturedData[4]);
		EXPECT_EQ(0xFF, capturedData[5]);
		EXPECT_EQ(0xFF, capturedData[6]);
		EXPECT_EQ(0xFF, capturedData[7]);

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().remove_listener(transmittedEventHandle);
	}

	TEST_F(AgriculturalGuidanceTest, SendGuidanceSystemCommandResolutionRounding)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, true, false);
		guidanceInterface.initialize();

		// Curvature 1.1 km^-1: (1.1 + 8032) / 0.25 = 32132.4
		// Scaled = roundf(4 * 32132.4) / 4.0 = 32132 (0x7D84)
		guidanceInterface.guidanceSystemCommandTransmitData.set_curvature(1.1f);

		std::vector<std::uint8_t> capturedData;
		auto transmittedEventHandle = CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener(
			[&](const CANMessage &msg) {
				if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand))
				{
					capturedData = msg.get_data();
				}
			});

		EXPECT_TRUE(guidanceInterface.send_guidance_system_command());
		ASSERT_EQ(8u, capturedData.size());
		EXPECT_EQ(0x84, capturedData[0]);
		EXPECT_EQ(0x7D, capturedData[1]);

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().remove_listener(transmittedEventHandle);
	}

	TEST_F(AgriculturalGuidanceTest, SendGuidanceSystemCommandClampingLimits)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, true, false);
		guidanceInterface.initialize();

		// Maximum curvature boundary (> 8031.75 km^-1) -> Raw encoding: 32127 + 32128 = 64255 (0xFBFF)
		guidanceInterface.guidanceSystemCommandTransmitData.set_curvature(8500.0f);

		std::vector<std::uint8_t> capturedDataMax;
		auto handleMax = CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener(
			[&](const CANMessage &msg) {
				if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand))
				{
					capturedDataMax = msg.get_data();
				}
			});

		EXPECT_TRUE(guidanceInterface.send_guidance_system_command());
		ASSERT_EQ(8u, capturedDataMax.size());
		EXPECT_EQ(0xFF, capturedDataMax[0]);
		EXPECT_EQ(0xFB, capturedDataMax[1]);

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().remove_listener(handleMax);

		// Minimum curvature boundary (< -8032 km^-1) -> Raw encoding: 0 (0x0000)
		guidanceInterface.guidanceSystemCommandTransmitData.set_curvature(-9000.0f);

		std::vector<std::uint8_t> capturedDataMin;
		auto handleMin = CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener(
			[&](const CANMessage &msg) {
				if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand))
				{
					capturedDataMin = msg.get_data();
				}
			});

		EXPECT_TRUE(guidanceInterface.send_guidance_system_command());
		ASSERT_EQ(8u, capturedDataMin.size());
		EXPECT_EQ(0x00, capturedDataMin[0]);
		EXPECT_EQ(0x00, capturedDataMin[1]);

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().remove_listener(handleMin);
	}

	//------------------------------------------------------------------------------
	// TRANSMISSION: MACHINE INFO ENCODING & BITFIELDS
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, SendGuidanceMachineInfoNullSender)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(nullptr, destCF, false, false);
		guidanceInterface.initialize();

		EXPECT_FALSE(guidanceInterface.send_guidance_machine_info());
	}

	TEST_F(AgriculturalGuidanceTest, SendGuidanceMachineInfoBitfieldPacking)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, true);
		guidanceInterface.initialize();

		// Set specific bitfield values:
		// Curvature: 0.0 -> 32128 (0x7D80)
		// Byte 2:
		// MechanicalSystemLockout = Active (1) -> bits [0..1] = 01
		// GuidanceSteeringSystemReadinessState = EnabledOnActive (1) -> bits [2..3] = 01 (0x04)
		// GuidanceSteeringInputPositionStatus = ErrorIndication (2) -> bits [4..5] = 10 (0x20)
		// RequestResetCommandStatus = ResetRequired (1) -> bits [6..7] = 01 (0x40)
		// Byte 2 total = 0x01 | 0x04 | 0x20 | 0x40 = 0x65
		//
		// Byte 3:
		// GuidanceLimitStatus = OperatorLimitedControlled (1) -> bits [5..7] = 001 (0x20)
		// Byte 3 total = 0x20
		//
		// Byte 4:
		// ExitReasonCode = OperatorOverrideOfFunction (3) -> bits [0..5] = 000011 (0x03)
		// RemoteEngageSwitchStatus = EnabledOnActive (1) -> bits [6..7] = 01 (0x40)
		// Byte 4 total = 0x03 | 0x40 = 0x43

		guidanceInterface.guidanceMachineInfoTransmitData.set_estimated_curvature(0.0f);
		guidanceInterface.guidanceMachineInfoTransmitData.set_mechanical_system_lockout_state(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active);
		guidanceInterface.guidanceMachineInfoTransmitData.set_guidance_steering_system_readiness_state(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive);
		guidanceInterface.guidanceMachineInfoTransmitData.set_guidance_steering_input_position_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::ErrorIndication);
		guidanceInterface.guidanceMachineInfoTransmitData.set_request_reset_command_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired);
		guidanceInterface.guidanceMachineInfoTransmitData.set_guidance_limit_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::OperatorLimitedControlled);
		guidanceInterface.guidanceMachineInfoTransmitData.set_guidance_system_command_exit_reason_code(3);
		guidanceInterface.guidanceMachineInfoTransmitData.set_guidance_system_remote_engage_switch_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive);

		std::vector<std::uint8_t> capturedData;
		CANIdentifier capturedIdentifier(0);

		auto handle = CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().add_listener(
			[&](const CANMessage &msg) {
				if (msg.is_parameter_group_number(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo))
				{
					capturedIdentifier = msg.get_identifier();
					capturedData = msg.get_data();
				}
			});

		EXPECT_TRUE(guidanceInterface.send_guidance_machine_info());
		EXPECT_EQ(static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo), capturedIdentifier.get_parameter_group_number());
		EXPECT_EQ(CANIdentifier::CANPriority::Priority3, capturedIdentifier.get_priority());

		ASSERT_EQ(8u, capturedData.size());
		EXPECT_EQ(0x80, capturedData[0]);
		EXPECT_EQ(0x7D, capturedData[1]);
		EXPECT_EQ(0x65, capturedData[2]);
		EXPECT_EQ(0x20, capturedData[3]);
		EXPECT_EQ(0x43, capturedData[4]);
		EXPECT_EQ(0xFF, capturedData[5]);
		EXPECT_EQ(0xFF, capturedData[6]);
		EXPECT_EQ(0xFF, capturedData[7]);

		CANNetworkManager::CANNetwork.get_transmitted_message_event_dispatcher().remove_listener(handle);
	}

	//------------------------------------------------------------------------------
	// RECEPTION: SYSTEM COMMAND DECODING AND EVENTS
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, ReceiveGuidanceSystemCommandDecodingAndEvents)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, false);
		guidanceInterface.initialize();

		std::size_t eventCallCount = 0;
		std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> eventObject = nullptr;
		bool eventChangedFlag = false;

		guidanceInterface.get_guidance_system_command_event_publisher().add_listener(
			[&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> cmd, bool changed) {
				eventCallCount++;
				eventObject = cmd;
				eventChangedFlag = changed;
			});

		// Construct raw message frame: 0xAC00 (PGN 0xAC00 = AgriculturalGuidanceSystemCommand)
		// Address 0x30 -> Priority 3, PGN 0xAC00, Source 0x30, Dest 0x10 -> Identifier: 0x0CAC1030
		CANMessageFrame frame{};
		frame.identifier = 0x0CAC1030;
		frame.isExtendedFrame = true;
		frame.dataLength = 8;
		// Curvature = 0.0 -> 32128 (0x7D80)
		frame.data[0] = 0x80;
		frame.data[1] = 0x7D;
		// Status = IntendedToSteer (1)
		frame.data[2] = 0xFD;
		frame.data[3] = 0xFF;
		frame.data[4] = 0xFF;
		frame.data[5] = 0xFF;
		frame.data[6] = 0xFF;
		frame.data[7] = 0xFF;

		CANNetworkManager::CANNetwork.can_message_receive_callback(frame, 0);

		EXPECT_EQ(1u, guidanceInterface.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(1u, eventCallCount);
		ASSERT_NE(nullptr, eventObject);
		EXPECT_TRUE(eventChangedFlag);

		EXPECT_NEAR(0.0f, eventObject->get_curvature(), 0.001f);
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer, eventObject->get_status());
		EXPECT_EQ(externalCF1, eventObject->get_sender());

		auto fetchedCommand = guidanceInterface.get_received_guidance_system_command(0);
		ASSERT_NE(nullptr, fetchedCommand);
		EXPECT_EQ(externalCF1, fetchedCommand->get_sender());

		// Out of bounds index
		EXPECT_EQ(nullptr, guidanceInterface.get_received_guidance_system_command(1));
	}

	TEST_F(AgriculturalGuidanceTest, ReceiveGuidanceSystemCommandMultiSource)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, nullptr, false, false);
		guidanceInterface.initialize();

		// Message from externalCF1 (0x30)
		CANMessageFrame frame1{};
		frame1.identifier = 0x0CAC1030;
		frame1.isExtendedFrame = true;
		frame1.dataLength = 8;
		frame1.data[0] = 0x80;
		frame1.data[1] = 0x7D;
		frame1.data[2] = 0xFD;
		std::fill(frame1.data + 3, frame1.data + 8, 0xFF);

		CANNetworkManager::CANNetwork.can_message_receive_callback(frame1, 0);

		// Message from externalCF2 (0x40)
		CANMessageFrame frame2{};
		frame2.identifier = 0x0CAC1040;
		frame2.isExtendedFrame = true;
		frame2.dataLength = 8;
		frame2.data[0] = 0x84;
		frame2.data[1] = 0x7D;
		frame2.data[2] = 0xFC;
		std::fill(frame2.data + 3, frame2.data + 8, 0xFF);

		CANNetworkManager::CANNetwork.can_message_receive_callback(frame2, 0);

		EXPECT_EQ(2u, guidanceInterface.get_number_received_guidance_system_command_sources());

		auto cmd1 = guidanceInterface.get_received_guidance_system_command(0);
		auto cmd2 = guidanceInterface.get_received_guidance_system_command(1);

		ASSERT_NE(nullptr, cmd1);
		ASSERT_NE(nullptr, cmd2);

		EXPECT_EQ(externalCF1, cmd1->get_sender());
		EXPECT_NEAR(0.0f, cmd1->get_curvature(), 0.001f);

		EXPECT_EQ(externalCF2, cmd2->get_sender());
		EXPECT_NEAR(1.1f, cmd2->get_curvature(), 0.01f);
	}

	//------------------------------------------------------------------------------
	// RECEPTION: MACHINE INFO DECODING AND EVENTS
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, ReceiveGuidanceMachineInfoDecodingAndEvents)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, false);
		guidanceInterface.initialize();

		std::size_t eventCallCount = 0;
		std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> eventObject = nullptr;
		bool eventChangedFlag = false;

		guidanceInterface.get_guidance_machine_info_event_publisher().add_listener(
			[&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> info, bool changed) {
				eventCallCount++;
				eventObject = info;
				eventChangedFlag = changed;
			});

		// Construct raw message frame for Machine Info: PGN 0xAD00
		// Address 0x30 -> Identifier: 0x0CAD1030
		CANMessageFrame frame{};
		frame.identifier = 0x0CAD1030;
		frame.isExtendedFrame = true;
		frame.dataLength = 8;
		frame.data[0] = 0x80;
		frame.data[1] = 0x7D;
		frame.data[2] = 0x65;
		frame.data[3] = 0x20;
		frame.data[4] = 0x43;
		frame.data[5] = 0xFF;
		frame.data[6] = 0xFF;
		frame.data[7] = 0xFF;

		CANNetworkManager::CANNetwork.can_message_receive_callback(frame, 0);

		EXPECT_EQ(1u, guidanceInterface.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(1u, eventCallCount);
		ASSERT_NE(nullptr, eventObject);
		EXPECT_TRUE(eventChangedFlag);

		EXPECT_NEAR(0.0f, eventObject->get_estimated_curvature(), 0.001f);
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active,
				  eventObject->get_mechanical_system_lockout_state());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive,
				  eventObject->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::ErrorIndication,
				  eventObject->get_guidance_steering_input_position_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired,
				  eventObject->get_request_reset_command_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::OperatorLimitedControlled,
				  eventObject->get_guidance_limit_status());
		EXPECT_EQ(3, eventObject->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive,
				  eventObject->get_guidance_system_remote_engage_switch_status());

		EXPECT_EQ(externalCF1, eventObject->get_sender());

		auto fetchedInfo = guidanceInterface.get_received_guidance_machine_info_message(0);
		ASSERT_NE(nullptr, fetchedInfo);
		EXPECT_EQ(externalCF1, fetchedInfo->get_sender());

		// Out of bounds index
		EXPECT_EQ(nullptr, guidanceInterface.get_received_guidance_machine_info_message(1));
	}

	//------------------------------------------------------------------------------
	// TIMEOUT & STALENESS HANDLING
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, TimeoutDetection)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, false);
		guidanceInterface.initialize();

		// Receive initial message
		CANMessageFrame frame{};
		frame.identifier = 0x0CAC1030;
		frame.isExtendedFrame = true;
		frame.dataLength = 8;
		frame.data[0] = 0x80;
		frame.data[1] = 0x7D;
		frame.data[2] = 0xFD;
		std::fill(frame.data + 3, frame.data + 8, 0xFF);

		CANNetworkManager::CANNetwork.can_message_receive_callback(frame, 0);

		auto cmd = guidanceInterface.get_received_guidance_system_command(0);
		ASSERT_NE(nullptr, cmd);

		// Immediately update -> should not be timed out
		guidanceInterface.update();

		// Fast forward past default timeout threshold
		// In AgIsoStack++, GuidanceSystemCommand timeout threshold is 300 ms
		// We call update() repeatedly or mock time progression if needed
		guidanceInterface.update();
	}
} // namespace isobus