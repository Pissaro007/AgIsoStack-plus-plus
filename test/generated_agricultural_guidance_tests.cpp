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

		static void process_received_message(const CANMessage &message, AgriculturalGuidanceInterface *target)
		{
			AgriculturalGuidanceInterface::process_rx_message(message, target);
		}
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

			srcICF = CANNetworkManager::CANNetwork.create_internal_control_function(sourceName, 0, 0x10);
			ASSERT_NE(nullptr, srcICF);
			ASSERT_TRUE(srcICF->get_address_valid())
			  << "ICF address: " << static_cast<unsigned>(srcICF->get_address())
			  << ", state: " << static_cast<int>(srcICF->get_current_state());
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

		EXPECT_TRUE(guidanceInterface.send_guidance_machine_info()); // Or send_guidance_system_command
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
			[&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> &cmd, bool changed) {
				eventCallCount++;
				eventObject = cmd;
				eventChangedFlag = changed;
			});

		// CAN Payload:
		// Bytes 0-1: 32128 (0x7D80) -> 0.0 km^-1
		// Byte 2: Status = IntendedToSteer (1) | 0xFC = 0xFD
		std::array<std::uint8_t, 8> dataPayload1 = { 0x80, 0x7D, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

		CANIdentifier identifier(
			CANIdentifier::Type::Extended,
			static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand),
			CANIdentifier::CANPriority::Priority3,
			0xFF,
			externalCF1->get_address());

		CANMessage rxMsg1(
			CANMessage::Type::Receive,
			identifier,
			dataPayload1.data(),
			dataPayload1.size(),
			externalCF1,
			nullptr,
			0);

		CANMessageFrame frame1{};
		frame1.channel = 0;
		frame1.identifier = identifier.get_identifier();
		frame1.dataLength = 8;
		std::copy(dataPayload1.begin(), dataPayload1.end(), frame1.data);

		CANMessage receivedMessage1(CANMessage::Type::Receive, identifier, dataPayload1.data(), dataPayload1.size(), externalCF1, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_received_message(receivedMessage1, &guidanceInterface);

		// Verify first message created source object
		EXPECT_EQ(1u, guidanceInterface.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(1u, eventCallCount);
		EXPECT_TRUE(eventChangedFlag);
		ASSERT_NE(nullptr, eventObject);
		EXPECT_EQ(externalCF1, eventObject->get_sender_control_function());
		EXPECT_NEAR(0.0f, eventObject->get_curvature(), 0.001f);
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer, eventObject->get_status());

		// Access via index
		auto fetchedObj = guidanceInterface.get_received_guidance_system_command(0);
		EXPECT_EQ(eventObject, fetchedObj);

		// Access out of bounds
		EXPECT_EQ(nullptr, guidanceInterface.get_received_guidance_system_command(1));

		// Repeat identical message -> changed should be false
		TestableAgriculturalGuidanceInterface::process_received_message(receivedMessage1, &guidanceInterface);

		EXPECT_EQ(1u, guidanceInterface.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(2u, eventCallCount);
		EXPECT_FALSE(eventChangedFlag);

		// Message with modified curvature only: 10.0 km^-1 -> (10.0 + 8032) / 0.25 = 32168 (0x7DA8)
		std::array<std::uint8_t, 8> dataPayload2 = { 0xA8, 0x7D, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
		CANMessageFrame frame2{};
		frame2.channel = 0;
		frame2.identifier = identifier.get_identifier();
		frame2.dataLength = 8;
		std::copy(dataPayload2.begin(), dataPayload2.end(), frame2.data);

		CANMessage receivedMessage2(CANMessage::Type::Receive, identifier, dataPayload2.data(), dataPayload2.size(), externalCF1, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_received_message(receivedMessage2, &guidanceInterface);

		EXPECT_EQ(3u, eventCallCount);
		EXPECT_TRUE(eventChangedFlag);
		EXPECT_NEAR(10.0f, eventObject->get_curvature(), 0.001f);
	}

	//------------------------------------------------------------------------------
	// RECEPTION: MACHINE INFO DECODING AND MULTI-SOURCE MANAGEMENT
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, ReceiveGuidanceMachineInfoBitfieldsAndMultipleSources)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, false);
		guidanceInterface.initialize();

		std::size_t eventCallCount = 0;
		std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> lastEventInfo = nullptr;
		bool lastChangedState = false;

		guidanceInterface.get_guidance_machine_info_event_publisher().add_listener(
			[&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> &info, bool changed) {
				eventCallCount++;
				lastEventInfo = info;
				lastChangedState = changed;
			});

		// Source 1 message
		// Byte 0-1: 32128 -> 0.0 km^-1
		// Byte 2: Lockout = Active (1), Readiness = EnabledOnActive (1), InputPos = DisabledOffPassive (0), Reset = ResetNotRequired (0) -> 0x05
		// Byte 3: LimitStatus = LimitedHigh (2) -> bits [5..7] = 010 -> 0x40
		// Byte 4: ExitReason = RemoteCommandTimeout (5) -> bits [0..5] = 000101, RemoteSwitch = ErrorIndication (2) -> bits [6..7] = 10 -> 0x85
		std::array<std::uint8_t, 8> payload1 = { 0x80, 0x7D, 0x05, 0x40, 0x85, 0xFF, 0xFF, 0xFF };

		CANIdentifier id1(
			CANIdentifier::Type::Extended,
			static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo),
			CANIdentifier::CANPriority::Priority3,
			0xFF,
			externalCF1->get_address());

		CANMessageFrame frame1{};
		frame1.channel = 0;
		frame1.identifier = id1.get_identifier();
		frame1.dataLength = 8;
		std::copy(payload1.begin(), payload1.end(), frame1.data);

		CANMessage receivedMachineInfo1(CANMessage::Type::Receive, id1, payload1.data(), payload1.size(), externalCF1, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_received_message(receivedMachineInfo1, &guidanceInterface);

		EXPECT_EQ(1u, guidanceInterface.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(1u, eventCallCount);
		EXPECT_TRUE(lastChangedState);
		ASSERT_NE(nullptr, lastEventInfo);

		EXPECT_EQ(externalCF1, lastEventInfo->get_sender_control_function());
		EXPECT_NEAR(0.0f, lastEventInfo->get_estimated_curvature(), 0.001f);
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active, lastEventInfo->get_mechanical_system_lockout());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive, lastEventInfo->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::DisabledOffPassive, lastEventInfo->get_guidance_steering_input_position_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetNotRequired, lastEventInfo->get_request_reset_command_status());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedHigh, lastEventInfo->get_guidance_limit_status());
		EXPECT_EQ(5, lastEventInfo->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::ErrorIndication, lastEventInfo->get_guidance_system_remote_engage_switch_status());

		// Source 2 message
		CANIdentifier id2(
			CANIdentifier::Type::Extended,
			static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo),
			CANIdentifier::CANPriority::Priority3,
			0xFF,
			externalCF2->get_address());

		CANMessageFrame frame2{};
		frame2.channel = 0;
		frame2.identifier = id2.get_identifier();
		frame2.dataLength = 8;
		std::copy(payload1.begin(), payload1.end(), frame2.data);

		CANMessage receivedMachineInfo2(CANMessage::Type::Receive, id2, payload1.data(), payload1.size(), externalCF2, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_received_message(receivedMachineInfo2, &guidanceInterface);

		EXPECT_EQ(2u, guidanceInterface.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(2u, eventCallCount);
		EXPECT_TRUE(lastChangedState); // First time seen from source 2 -> changed is true
		EXPECT_EQ(externalCF2, lastEventInfo->get_sender_control_function());

		// Index verification
		EXPECT_EQ(externalCF1, guidanceInterface.get_received_guidance_machine_info(0)->get_sender_control_function());
		EXPECT_EQ(externalCF2, guidanceInterface.get_received_guidance_machine_info(1)->get_sender_control_function());
		EXPECT_EQ(nullptr, guidanceInterface.get_received_guidance_machine_info(2));
	}

	//------------------------------------------------------------------------------
	// RECEPTION ERROR HANDLING & MALFORMED MESSAGES
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, ReceiveMalformedAndInvalidMessages)
	{
		TestableAgriculturalGuidanceInterface guidanceInterface(srcICF, destCF, false, false);
		guidanceInterface.initialize();

		// DLC != 8 should be rejected
		std::array<std::uint8_t, 7> shortPayload = { 0x80, 0x7D, 0x00, 0x00, 0x00, 0x00, 0x00 };
		CANIdentifier id(
			CANIdentifier::Type::Extended,
			static_cast<std::uint32_t>(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand),
			CANIdentifier::CANPriority::Priority3,
			0xFF,
			externalCF1->get_address());

		CANMessageFrame frameShort{};
		frameShort.channel = 0;
		frameShort.identifier = id.get_identifier();
		frameShort.dataLength = 7;
		std::copy(shortPayload.begin(), shortPayload.end(), frameShort.data);

		CANMessage malformedMessage(CANMessage::Type::Receive, id, shortPayload.data(), shortPayload.size(), externalCF1, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_received_message(malformedMessage, &guidanceInterface);

		EXPECT_EQ(0u, guidanceInterface.get_number_received_guidance_system_command_sources());

		// Unhandled PGN should have no effect
		CANIdentifier unhandledId(
			CANIdentifier::Type::Extended,
			static_cast<std::uint32_t>(CANLibParameterGroupNumber::VirtualTerminalToECU),
			CANIdentifier::CANPriority::Priority3,
			0xFF,
			externalCF1->get_address());

		std::array<std::uint8_t, 8> validPayload = { 0x80, 0x7D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
		CANMessageFrame frameUnhandled{};
		frameUnhandled.channel = 0;
		frameUnhandled.identifier = unhandledId.get_identifier();
		frameUnhandled.dataLength = 8;
		std::copy(validPayload.begin(), validPayload.end(), frameUnhandled.data);

		CANMessage unhandledMessage(CANMessage::Type::Receive, unhandledId, validPayload.data(), validPayload.size(), externalCF1, nullptr, 0);
		TestableAgriculturalGuidanceInterface::process_received_message(unhandledMessage, &guidanceInterface);

		EXPECT_EQ(0u, guidanceInterface.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(0u, guidanceInterface.get_number_received_guidance_machine_info_message_sources());
	}

	//------------------------------------------------------------------------------
	// INTERNAL DATA CLASS SETTERS AND GETTERS
	//------------------------------------------------------------------------------

	TEST_F(AgriculturalGuidanceTest, GuidanceSystemCommandSettersAndGetters)
	{
		AgriculturalGuidanceInterface::GuidanceSystemCommand cmd(externalCF1);

		EXPECT_EQ(externalCF1, cmd.get_sender_control_function());
		EXPECT_EQ(0.0f, cmd.get_curvature());
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::NotAvailable, cmd.get_status());

		// Status changes
		EXPECT_TRUE(cmd.set_status(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer));
		EXPECT_FALSE(cmd.set_status(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceSystemCommand::CurvatureCommandStatus::IntendedToSteer, cmd.get_status());

		// Curvature changes
		EXPECT_TRUE(cmd.set_curvature(5.25f));
		EXPECT_FALSE(cmd.set_curvature(5.25f));
		EXPECT_FLOAT_EQ(5.25f, cmd.get_curvature());

		// Timestamp
		cmd.set_timestamp_ms(12345);
		EXPECT_EQ(12345u, cmd.get_timestamp_ms());
	}

	TEST_F(AgriculturalGuidanceTest, GuidanceMachineInfoSettersAndGetters)
	{
		AgriculturalGuidanceInterface::GuidanceMachineInfo info(externalCF1);

		EXPECT_EQ(externalCF1, info.get_sender_control_function());

		// Curvature
		EXPECT_TRUE(info.set_estimated_curvature(-12.5f));
		EXPECT_FALSE(info.set_estimated_curvature(-12.5f));
		EXPECT_FLOAT_EQ(-12.5f, info.get_estimated_curvature());

		// Mechanical system lockout
		EXPECT_TRUE(info.set_mechanical_system_lockout_state(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active));
		EXPECT_FALSE(info.set_mechanical_system_lockout_state(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::MechanicalSystemLockout::Active, info.get_mechanical_system_lockout());

		// Steering readiness
		EXPECT_TRUE(info.set_guidance_steering_system_readiness_state(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_FALSE(info.set_guidance_steering_system_readiness_state(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive, info.get_guidance_steering_system_readiness_state());

		// Input position status
		EXPECT_TRUE(info.set_guidance_steering_input_position_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::ErrorIndication));
		EXPECT_FALSE(info.set_guidance_steering_input_position_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::ErrorIndication));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::ErrorIndication, info.get_guidance_steering_input_position_status());

		// Request reset command status
		EXPECT_TRUE(info.set_request_reset_command_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired));
		EXPECT_FALSE(info.set_request_reset_command_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::RequestResetCommandStatus::ResetRequired, info.get_request_reset_command_status());

		// Guidance limit status
		EXPECT_TRUE(info.set_guidance_limit_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedLow));
		EXPECT_FALSE(info.set_guidance_limit_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedLow));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GuidanceLimitStatus::LimitedLow, info.get_guidance_limit_status());

		// Exit reason code
		EXPECT_TRUE(info.set_guidance_system_command_exit_reason_code(23));
		EXPECT_FALSE(info.set_guidance_system_command_exit_reason_code(23));
		EXPECT_EQ(23, info.get_guidance_system_command_exit_reason_code());

		// Remote engage switch status
		EXPECT_TRUE(info.set_guidance_system_remote_engage_switch_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_FALSE(info.set_guidance_system_remote_engage_switch_status(
			AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_EQ(AgriculturalGuidanceInterface::GuidanceMachineInfo::GenericSAEbs02SlotValue::EnabledOnActive, info.get_guidance_system_remote_engage_switch_status());

		// Timestamp
		info.set_timestamp_ms(99999);
		EXPECT_EQ(99999u, info.get_timestamp_ms());
	}
} // namespace isobus