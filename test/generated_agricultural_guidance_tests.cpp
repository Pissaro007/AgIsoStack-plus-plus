#include "isobus/isobus/can_control_function.hpp"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_identifier.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/isobus_guidance_interface.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/utility/time_source.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace
{
	using namespace isobus;

	class ManualTimeSource : public TimeSource
	{
	public:
		std::uint32_t get_current_time_ms() const override
		{
			return currentTime_ms;
		}

		std::uint64_t get_current_time_us() const override
		{
			return static_cast<std::uint64_t>(currentTime_ms) * 1000ULL;
		}

		void set_time_ms(std::uint32_t time_ms)
		{
			currentTime_ms = time_ms;
		}

	private:
		std::uint32_t currentTime_ms = 0;
	};

	class TestableAgriculturalGuidanceInterface : public AgriculturalGuidanceInterface
	{
	public:
		TestableAgriculturalGuidanceInterface(std::shared_ptr<InternalControlFunction> source = nullptr,
		                                     std::shared_ptr<ControlFunction> destination = nullptr,
		                                     bool enableSendingSystemCommandPeriodically = false,
		                                     bool enableSendingMachineInfoPeriodically = false) :
		  AgriculturalGuidanceInterface(source,
		                                destination,
		                                enableSendingSystemCommandPeriodically,
		                                enableSendingMachineInfoPeriodically)
		{
		}

		void process_received_message(const CANMessage &message)
		{
			process_rx_message(message, this);
		}

		bool send_machine_info() const
		{
			return send_guidance_machine_info();
		}

		bool send_system_command() const
		{
			return send_guidance_system_command();
		}
	};

	class AgriculturalGuidanceGeneratedTest : public testing::Test
	{
	protected:
		void SetUp() override
		{
			timeSource.set_time_ms(1000);
			SystemTiming::override_time_source(&timeSource);

			sourceOne = std::make_shared<ControlFunction>(
			  NAME(0x0102030405060708ULL),
			  0x31,
			  0,
			  ControlFunction::Type::External);

			sourceTwo = std::make_shared<ControlFunction>(
			  NAME(0x1112131415161718ULL),
			  0x32,
			  0,
			  ControlFunction::Type::External);
		}

		void TearDown() override
		{
			SystemTiming::override_time_source(nullptr);
		}

		static CANIdentifier make_identifier(CANLibParameterGroupNumber pgn,
		                                     std::uint8_t sourceAddress)
		{
			return CANIdentifier(
			  CANIdentifier::Type::Extended,
			  static_cast<std::uint32_t>(pgn),
			  CANIdentifier::CANPriority::Priority3,
			  CANIdentifier::GLOBAL_ADDRESS,
			  sourceAddress);
		}

		static CANMessage make_message(CANLibParameterGroupNumber pgn,
		                               const std::vector<std::uint8_t> &data,
		                               const std::shared_ptr<ControlFunction> &source)
		{
			const std::uint8_t sourceAddress =
			  (nullptr != source) ? source->get_address() : CANIdentifier::NULL_ADDRESS;

			return CANMessage(
			  CANMessage::Type::Receive,
			  make_identifier(pgn, sourceAddress),
			  data,
			  source,
			  nullptr,
			  0);
		}

		ManualTimeSource timeSource;
		std::shared_ptr<ControlFunction> sourceOne;
		std::shared_ptr<ControlFunction> sourceTwo;
	};

	TEST_F(AgriculturalGuidanceGeneratedTest, InitializationIsIdempotentAndInitialStateIsEmpty)
	{
		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		EXPECT_FALSE(interfaceUnderTest.get_initialized());
		EXPECT_EQ(0U, interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(0U, interfaceUnderTest.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(nullptr, interfaceUnderTest.get_received_guidance_machine_info(0));
		EXPECT_EQ(nullptr, interfaceUnderTest.get_received_guidance_system_command(0));

		interfaceUnderTest.initialize();

		EXPECT_TRUE(interfaceUnderTest.get_initialized());

		interfaceUnderTest.initialize();

		EXPECT_TRUE(interfaceUnderTest.get_initialized());
		EXPECT_EQ(0U, interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(0U, interfaceUnderTest.get_number_received_guidance_system_command_sources());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, SendingIsRejectedWithoutConfiguredTransmittingControlFunction)
	{
		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		EXPECT_FALSE(interfaceUnderTest.send_machine_info());
		EXPECT_FALSE(interfaceUnderTest.send_system_command());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, GuidanceSystemCommandSettersReportOnlyRealChanges)
	{
		using Command = AgriculturalGuidanceInterface::GuidanceSystemCommand;

		Command command(sourceOne);

		EXPECT_EQ(sourceOne, command.get_sender_control_function());
		EXPECT_FLOAT_EQ(0.0F, command.get_curvature());
		EXPECT_EQ(Command::CurvatureCommandStatus::NotAvailable, command.get_status());
		EXPECT_EQ(0U, command.get_timestamp_ms());

		EXPECT_FALSE(command.set_curvature(0.0F));
		EXPECT_TRUE(command.set_curvature(12.25F));
		EXPECT_FLOAT_EQ(12.25F, command.get_curvature());
		EXPECT_FALSE(command.set_curvature(12.25F));
		EXPECT_TRUE(command.set_curvature(-8032.0F));
		EXPECT_FLOAT_EQ(-8032.0F, command.get_curvature());

		EXPECT_FALSE(command.set_status(Command::CurvatureCommandStatus::NotAvailable));
		EXPECT_TRUE(command.set_status(Command::CurvatureCommandStatus::IntendedToSteer));
		EXPECT_EQ(Command::CurvatureCommandStatus::IntendedToSteer, command.get_status());
		EXPECT_FALSE(command.set_status(Command::CurvatureCommandStatus::IntendedToSteer));
		EXPECT_TRUE(command.set_status(Command::CurvatureCommandStatus::Error));
		EXPECT_EQ(Command::CurvatureCommandStatus::Error, command.get_status());

		command.set_timestamp_ms(4567U);
		EXPECT_EQ(4567U, command.get_timestamp_ms());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, GuidanceMachineInfoSettersReportOnlyRealChanges)
	{
		using Info = AgriculturalGuidanceInterface::GuidanceMachineInfo;

		Info info(sourceOne);

		EXPECT_EQ(sourceOne, info.get_sender_control_function());
		EXPECT_FLOAT_EQ(0.0F, info.get_estimated_curvature());
		EXPECT_EQ(Info::MechanicalSystemLockout::NotAvailable,
		          info.get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction,
		          info.get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction,
		          info.get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::NotAvailable,
		          info.get_request_reset_command_status());
		EXPECT_EQ(Info::GuidanceLimitStatus::NotAvailable,
		          info.get_guidance_limit_status());
		EXPECT_EQ(63U, info.get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction,
		          info.get_guidance_system_remote_engage_switch_status());
		EXPECT_EQ(0U, info.get_timestamp_ms());

		EXPECT_FALSE(info.set_estimated_curvature(0.0F));
		EXPECT_TRUE(info.set_estimated_curvature(-0.25F));
		EXPECT_FLOAT_EQ(-0.25F, info.get_estimated_curvature());
		EXPECT_FALSE(info.set_estimated_curvature(-0.25F));

		EXPECT_FALSE(info.set_mechanical_system_lockout_state(
		  Info::MechanicalSystemLockout::NotAvailable));
		EXPECT_TRUE(info.set_mechanical_system_lockout_state(
		  Info::MechanicalSystemLockout::Active));
		EXPECT_EQ(Info::MechanicalSystemLockout::Active,
		          info.get_mechanical_system_lockout());
		EXPECT_FALSE(info.set_mechanical_system_lockout_state(
		  Info::MechanicalSystemLockout::Active));

		EXPECT_FALSE(info.set_guidance_steering_system_readiness_state(
		  Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction));
		EXPECT_TRUE(info.set_guidance_steering_system_readiness_state(
		  Info::GenericSAEbs02SlotValue::EnabledOnActive));
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          info.get_guidance_steering_system_readiness_state());
		EXPECT_FALSE(info.set_guidance_steering_system_readiness_state(
		  Info::GenericSAEbs02SlotValue::EnabledOnActive));

		EXPECT_FALSE(info.set_guidance_steering_input_position_status(
		  Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction));
		EXPECT_TRUE(info.set_guidance_steering_input_position_status(
		  Info::GenericSAEbs02SlotValue::ErrorIndication));
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::ErrorIndication,
		          info.get_guidance_steering_input_position_status());
		EXPECT_FALSE(info.set_guidance_steering_input_position_status(
		  Info::GenericSAEbs02SlotValue::ErrorIndication));

		EXPECT_FALSE(info.set_request_reset_command_status(
		  Info::RequestResetCommandStatus::NotAvailable));
		EXPECT_TRUE(info.set_request_reset_command_status(
		  Info::RequestResetCommandStatus::ResetRequired));
		EXPECT_EQ(Info::RequestResetCommandStatus::ResetRequired,
		          info.get_request_reset_command_status());
		EXPECT_FALSE(info.set_request_reset_command_status(
		  Info::RequestResetCommandStatus::ResetRequired));

		EXPECT_FALSE(info.set_guidance_limit_status(
		  Info::GuidanceLimitStatus::NotAvailable));
		EXPECT_TRUE(info.set_guidance_limit_status(
		  Info::GuidanceLimitStatus::LimitedHigh));
		EXPECT_EQ(Info::GuidanceLimitStatus::LimitedHigh,
		          info.get_guidance_limit_status());
		EXPECT_FALSE(info.set_guidance_limit_status(
		  Info::GuidanceLimitStatus::LimitedHigh));

		EXPECT_FALSE(info.set_guidance_system_command_exit_reason_code(63U));
		EXPECT_TRUE(info.set_guidance_system_command_exit_reason_code(26U));
		EXPECT_EQ(26U, info.get_guidance_system_command_exit_reason_code());
		EXPECT_FALSE(info.set_guidance_system_command_exit_reason_code(26U));

		EXPECT_FALSE(info.set_guidance_system_remote_engage_switch_status(
		  Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction));
		EXPECT_TRUE(info.set_guidance_system_remote_engage_switch_status(
		  Info::GenericSAEbs02SlotValue::DisabledOffPassive));
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::DisabledOffPassive,
		          info.get_guidance_system_remote_engage_switch_status());
		EXPECT_FALSE(info.set_guidance_system_remote_engage_switch_status(
		  Info::GenericSAEbs02SlotValue::DisabledOffPassive));

		info.set_timestamp_ms(9876U);
		EXPECT_EQ(9876U, info.get_timestamp_ms());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, SystemCommandDecodesCurvatureStatusAndReportsChanged)
	{
		using Command = AgriculturalGuidanceInterface::GuidanceSystemCommand;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::size_t callbackCount = 0;
		std::shared_ptr<Command> callbackObject;
		bool callbackChanged = false;

		interfaceUnderTest.get_guidance_system_command_event_publisher().add_listener(
		  [&](const std::shared_ptr<Command> &command, const bool &changed) {
			  ++callbackCount;
			  callbackObject = command;
			  callbackChanged = changed;
		  });

		const std::vector<std::uint8_t> data = {
			0x7F,
			0x7D,
			0xFE,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               data,
		               sourceOne));

		ASSERT_EQ(1U, callbackCount);
		ASSERT_NE(nullptr, callbackObject);
		EXPECT_TRUE(callbackChanged);
		EXPECT_EQ(sourceOne, callbackObject->get_sender_control_function());
		EXPECT_FLOAT_EQ(-0.25F, callbackObject->get_curvature());
		EXPECT_EQ(Command::CurvatureCommandStatus::Error, callbackObject->get_status());
		EXPECT_EQ(1000U, callbackObject->get_timestamp_ms());

		ASSERT_EQ(1U, interfaceUnderTest.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(callbackObject,
		          interfaceUnderTest.get_received_guidance_system_command(0));
		EXPECT_EQ(nullptr,
		          interfaceUnderTest.get_received_guidance_system_command(1));
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, IdenticalSystemCommandReusesObjectAndReportsUnchanged)
	{
		using Command = AgriculturalGuidanceInterface::GuidanceSystemCommand;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::vector<std::shared_ptr<Command>> callbackObjects;
		std::vector<bool> changedValues;

		interfaceUnderTest.get_guidance_system_command_event_publisher().add_listener(
		  [&](const std::shared_ptr<Command> &command, const bool &changed) {
			  callbackObjects.push_back(command);
			  changedValues.push_back(changed);
		  });

		const std::vector<std::uint8_t> data = {
			0x80,
			0x7D,
			0xFD,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               data,
		               sourceOne));

		timeSource.set_time_ms(1010);

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               data,
		               sourceOne));

		ASSERT_EQ(2U, callbackObjects.size());
		ASSERT_EQ(2U, changedValues.size());
		EXPECT_TRUE(changedValues[0]);
		EXPECT_FALSE(changedValues[1]);
		EXPECT_EQ(callbackObjects[0], callbackObjects[1]);
		EXPECT_EQ(sourceOne, callbackObjects[1]->get_sender_control_function());
		EXPECT_FLOAT_EQ(0.0F, callbackObjects[1]->get_curvature());
		EXPECT_EQ(Command::CurvatureCommandStatus::IntendedToSteer,
		          callbackObjects[1]->get_status());
		EXPECT_EQ(1010U, callbackObjects[1]->get_timestamp_ms());
		EXPECT_EQ(1U, interfaceUnderTest.get_number_received_guidance_system_command_sources());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, SystemCommandSingleFieldModificationReportsChanged)
	{
		using Command = AgriculturalGuidanceInterface::GuidanceSystemCommand;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::vector<bool> changedValues;
		std::vector<std::shared_ptr<Command>> callbackObjects;

		interfaceUnderTest.get_guidance_system_command_event_publisher().add_listener(
		  [&](const std::shared_ptr<Command> &command, const bool &changed) {
			  callbackObjects.push_back(command);
			  changedValues.push_back(changed);
		  });

		const std::vector<std::uint8_t> initialData = {
			0x80,
			0x7D,
			0xFC,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> modifiedData = {
			0x81,
			0x7D,
			0xFC,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               initialData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               modifiedData,
		               sourceOne));

		ASSERT_EQ(2U, callbackObjects.size());
		ASSERT_EQ(2U, changedValues.size());
		EXPECT_TRUE(changedValues[0]);
		EXPECT_TRUE(changedValues[1]);
		EXPECT_EQ(callbackObjects[0], callbackObjects[1]);
		EXPECT_FLOAT_EQ(0.25F, callbackObjects[1]->get_curvature());
		EXPECT_EQ(Command::CurvatureCommandStatus::NotIntendedToSteer,
		          callbackObjects[1]->get_status());
		EXPECT_EQ(1U, interfaceUnderTest.get_number_received_guidance_system_command_sources());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, SystemCommandsFromDifferentSourcesAreStoredIndependently)
	{
		using Command = AgriculturalGuidanceInterface::GuidanceSystemCommand;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::size_t callbackCount = 0;
		std::vector<std::shared_ptr<Command>> callbackObjects;
		std::vector<bool> changedValues;

		interfaceUnderTest.get_guidance_system_command_event_publisher().add_listener(
		  [&](const std::shared_ptr<Command> &command, const bool &changed) {
			  ++callbackCount;
			  callbackObjects.push_back(command);
			  changedValues.push_back(changed);
		  });

		const std::vector<std::uint8_t> sourceOneData = {
			0x84,
			0x7D,
			0xFD,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> sourceTwoData = {
			0x78,
			0x7D,
			0xFE,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               sourceOneData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               sourceTwoData,
		               sourceTwo));

		ASSERT_EQ(2U, callbackCount);
		ASSERT_EQ(2U, callbackObjects.size());
		ASSERT_EQ(2U, changedValues.size());
		EXPECT_TRUE(changedValues[0]);
		EXPECT_TRUE(changedValues[1]);
		EXPECT_NE(callbackObjects[0], callbackObjects[1]);

		EXPECT_EQ(2U, interfaceUnderTest.get_number_received_guidance_system_command_sources());

		const auto first = interfaceUnderTest.get_received_guidance_system_command(0);
		const auto second = interfaceUnderTest.get_received_guidance_system_command(1);

		ASSERT_NE(nullptr, first);
		ASSERT_NE(nullptr, second);
		EXPECT_EQ(sourceOne, first->get_sender_control_function());
		EXPECT_FLOAT_EQ(1.0F, first->get_curvature());
		EXPECT_EQ(Command::CurvatureCommandStatus::IntendedToSteer, first->get_status());
		EXPECT_EQ(sourceTwo, second->get_sender_control_function());
		EXPECT_FLOAT_EQ(-2.0F, second->get_curvature());
		EXPECT_EQ(Command::CurvatureCommandStatus::Error, second->get_status());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, MalformedAndSourceLessSystemCommandsHaveNoEffect)
	{
		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::size_t callbackCount = 0;

		interfaceUnderTest.get_guidance_system_command_event_publisher().add_listener(
		  [&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> &,
		      const bool &) {
			  ++callbackCount;
		  });

		const std::vector<std::uint8_t> malformedData = {
			0x80,
			0x7D,
			0xFC,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> validLengthData = {
			0x80,
			0x7D,
			0xFC,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               malformedData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               validLengthData,
		               nullptr));

		EXPECT_EQ(0U, callbackCount);
		EXPECT_EQ(0U, interfaceUnderTest.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(nullptr, interfaceUnderTest.get_received_guidance_system_command(0));
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, MachineInfoDecodesEveryBinaryFieldPrecisely)
	{
		using Info = AgriculturalGuidanceInterface::GuidanceMachineInfo;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::size_t callbackCount = 0;
		std::shared_ptr<Info> callbackObject;
		bool callbackChanged = false;

		interfaceUnderTest.get_guidance_machine_info_event_publisher().add_listener(
		  [&](const std::shared_ptr<Info> &info, const bool &changed) {
			  ++callbackCount;
			  callbackObject = info;
			  callbackChanged = changed;
		  });

		const std::vector<std::uint8_t> data = {
			0x82,
			0x7D,
			0xE7,
			0xB7,
			0x9A,
			0x12,
			0x34,
			0x56
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               data,
		               sourceOne));

		ASSERT_EQ(1U, callbackCount);
		ASSERT_NE(nullptr, callbackObject);
		EXPECT_TRUE(callbackChanged);
		EXPECT_EQ(sourceOne, callbackObject->get_sender_control_function());
		EXPECT_FLOAT_EQ(0.5F, callbackObject->get_estimated_curvature());
		EXPECT_EQ(Info::MechanicalSystemLockout::NotAvailable,
		          callbackObject->get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          callbackObject->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::ErrorIndication,
		          callbackObject->get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::NotAvailable,
		          callbackObject->get_request_reset_command_status());
		EXPECT_EQ(Info::GuidanceLimitStatus::Reserved_2,
		          callbackObject->get_guidance_limit_status());
		EXPECT_EQ(26U,
		          callbackObject->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::ErrorIndication,
		          callbackObject->get_guidance_system_remote_engage_switch_status());
		EXPECT_EQ(1000U, callbackObject->get_timestamp_ms());

		ASSERT_EQ(1U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(callbackObject,
		          interfaceUnderTest.get_received_guidance_machine_info(0));
		EXPECT_EQ(nullptr,
		          interfaceUnderTest.get_received_guidance_machine_info(1));
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, MachineInfoMasksReservedBitsInLimitStatusByte)
	{
		using Info = AgriculturalGuidanceInterface::GuidanceMachineInfo;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		const std::vector<std::uint8_t> data = {
			0x80,
			0x7D,
			0x00,
			0xDF,
			0x00,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               data,
		               sourceOne));

		const auto info = interfaceUnderTest.get_received_guidance_machine_info(0);

		ASSERT_NE(nullptr, info);
		EXPECT_EQ(Info::GuidanceLimitStatus::NonRecoverableFault,
		          info->get_guidance_limit_status());
		EXPECT_EQ(Info::MechanicalSystemLockout::NotActive,
		          info->get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::DisabledOffPassive,
		          info->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::DisabledOffPassive,
		          info->get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::ResetNotRequired,
		          info->get_request_reset_command_status());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, IdenticalMachineInfoReusesObjectAndReportsUnchanged)
	{
		using Info = AgriculturalGuidanceInterface::GuidanceMachineInfo;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::vector<std::shared_ptr<Info>> callbackObjects;
		std::vector<bool> changedValues;

		interfaceUnderTest.get_guidance_machine_info_event_publisher().add_listener(
		  [&](const std::shared_ptr<Info> &info, const bool &changed) {
			  callbackObjects.push_back(info);
			  changedValues.push_back(changed);
		  });

		const std::vector<std::uint8_t> data = {
			0x80,
			0x7D,
			0x1B,
			0x40,
			0x45,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               data,
		               sourceOne));

		timeSource.set_time_ms(1025);

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               data,
		               sourceOne));

		ASSERT_EQ(2U, callbackObjects.size());
		ASSERT_EQ(2U, changedValues.size());
		EXPECT_TRUE(changedValues[0]);
		EXPECT_FALSE(changedValues[1]);
		EXPECT_EQ(callbackObjects[0], callbackObjects[1]);
		EXPECT_EQ(sourceOne, callbackObjects[1]->get_sender_control_function());
		EXPECT_FLOAT_EQ(0.0F, callbackObjects[1]->get_estimated_curvature());
		EXPECT_EQ(Info::MechanicalSystemLockout::NotAvailable,
		          callbackObjects[1]->get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::ErrorIndication,
		          callbackObjects[1]->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          callbackObjects[1]->get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::ResetNotRequired,
		          callbackObjects[1]->get_request_reset_command_status());
		EXPECT_EQ(Info::GuidanceLimitStatus::LimitedHigh,
		          callbackObjects[1]->get_guidance_limit_status());
		EXPECT_EQ(5U,
		          callbackObjects[1]->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          callbackObjects[1]->get_guidance_system_remote_engage_switch_status());
		EXPECT_EQ(1025U, callbackObjects[1]->get_timestamp_ms());
		EXPECT_EQ(1U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, MachineInfoSingleFieldModificationReportsChanged)
	{
		using Info = AgriculturalGuidanceInterface::GuidanceMachineInfo;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::vector<std::shared_ptr<Info>> callbackObjects;
		std::vector<bool> changedValues;

		interfaceUnderTest.get_guidance_machine_info_event_publisher().add_listener(
		  [&](const std::shared_ptr<Info> &info, const bool &changed) {
			  callbackObjects.push_back(info);
			  changedValues.push_back(changed);
		  });

		const std::vector<std::uint8_t> initialData = {
			0x80,
			0x7D,
			0xE4,
			0x60,
			0xCA,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> modifiedData = {
			0x80,
			0x7D,
			0xE5,
			0x60,
			0xCA,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               initialData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               modifiedData,
		               sourceOne));

		ASSERT_EQ(2U, callbackObjects.size());
		ASSERT_EQ(2U, changedValues.size());
		EXPECT_TRUE(changedValues[0]);
		EXPECT_TRUE(changedValues[1]);
		EXPECT_EQ(callbackObjects[0], callbackObjects[1]);
		EXPECT_EQ(Info::MechanicalSystemLockout::Active,
		          callbackObjects[1]->get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          callbackObjects[1]->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::ErrorIndication,
		          callbackObjects[1]->get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::NotAvailable,
		          callbackObjects[1]->get_request_reset_command_status());
		EXPECT_EQ(Info::GuidanceLimitStatus::LimitedLow,
		          callbackObjects[1]->get_guidance_limit_status());
		EXPECT_EQ(10U,
		          callbackObjects[1]->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction,
		          callbackObjects[1]->get_guidance_system_remote_engage_switch_status());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, MachineInfoMultipleFieldModificationReportsChanged)
	{
		using Info = AgriculturalGuidanceInterface::GuidanceMachineInfo;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::vector<bool> changedValues;

		interfaceUnderTest.get_guidance_machine_info_event_publisher().add_listener(
		  [&](const std::shared_ptr<Info> &, const bool &changed) {
			  changedValues.push_back(changed);
		  });

		const std::vector<std::uint8_t> initialData = {
			0x80,
			0x7D,
			0x00,
			0x00,
			0x00,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> modifiedData = {
			0x7C,
			0x7D,
			0x9E,
			0xE0,
			0x7F,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               initialData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               modifiedData,
		               sourceOne));

		ASSERT_EQ(2U, changedValues.size());
		EXPECT_TRUE(changedValues[0]);
		EXPECT_TRUE(changedValues[1]);

		const auto info = interfaceUnderTest.get_received_guidance_machine_info(0);

		ASSERT_NE(nullptr, info);
		EXPECT_FLOAT_EQ(-1.0F, info->get_estimated_curvature());
		EXPECT_EQ(Info::MechanicalSystemLockout::Error,
		          info->get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction,
		          info->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          info->get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::Error,
		          info->get_request_reset_command_status());
		EXPECT_EQ(Info::GuidanceLimitStatus::NotAvailable,
		          info->get_guidance_limit_status());
		EXPECT_EQ(63U, info->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          info->get_guidance_system_remote_engage_switch_status());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, MachineInfoFromDifferentSourcesIsStoredIndependently)
	{
		using Info = AgriculturalGuidanceInterface::GuidanceMachineInfo;

		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::vector<std::shared_ptr<Info>> callbackObjects;
		std::vector<bool> changedValues;

		interfaceUnderTest.get_guidance_machine_info_event_publisher().add_listener(
		  [&](const std::shared_ptr<Info> &info, const bool &changed) {
			  callbackObjects.push_back(info);
			  changedValues.push_back(changed);
		  });

		const std::vector<std::uint8_t> sourceOneData = {
			0x84,
			0x7D,
			0x39,
			0x20,
			0x54,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> sourceTwoData = {
			0x78,
			0x7D,
			0xC6,
			0xC0,
			0xAB,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               sourceOneData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               sourceTwoData,
		               sourceTwo));

		ASSERT_EQ(2U, callbackObjects.size());
		ASSERT_EQ(2U, changedValues.size());
		EXPECT_TRUE(changedValues[0]);
		EXPECT_TRUE(changedValues[1]);
		EXPECT_NE(callbackObjects[0], callbackObjects[1]);

		EXPECT_EQ(2U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());

		const auto first = interfaceUnderTest.get_received_guidance_machine_info(0);
		const auto second = interfaceUnderTest.get_received_guidance_machine_info(1);

		ASSERT_NE(nullptr, first);
		ASSERT_NE(nullptr, second);

		EXPECT_EQ(sourceOne, first->get_sender_control_function());
		EXPECT_FLOAT_EQ(1.0F, first->get_estimated_curvature());
		EXPECT_EQ(Info::MechanicalSystemLockout::Active,
		          first->get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::ErrorIndication,
		          first->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::NotAvailableTakeNoAction,
		          first->get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::ResetNotRequired,
		          first->get_request_reset_command_status());
		EXPECT_EQ(Info::GuidanceLimitStatus::OperatorLimitedControlled,
		          first->get_guidance_limit_status());
		EXPECT_EQ(20U, first->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          first->get_guidance_system_remote_engage_switch_status());

		EXPECT_EQ(sourceTwo, second->get_sender_control_function());
		EXPECT_FLOAT_EQ(-2.0F, second->get_estimated_curvature());
		EXPECT_EQ(Info::MechanicalSystemLockout::Error,
		          second->get_mechanical_system_lockout());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::EnabledOnActive,
		          second->get_guidance_steering_system_readiness_state());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::DisabledOffPassive,
		          second->get_guidance_steering_input_position_status());
		EXPECT_EQ(Info::RequestResetCommandStatus::NotAvailable,
		          second->get_request_reset_command_status());
		EXPECT_EQ(Info::GuidanceLimitStatus::NonRecoverableFault,
		          second->get_guidance_limit_status());
		EXPECT_EQ(43U, second->get_guidance_system_command_exit_reason_code());
		EXPECT_EQ(Info::GenericSAEbs02SlotValue::ErrorIndication,
		          second->get_guidance_system_remote_engage_switch_status());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, MalformedAndSourceLessMachineInfoMessagesHaveNoEffect)
	{
		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::size_t callbackCount = 0;

		interfaceUnderTest.get_guidance_machine_info_event_publisher().add_listener(
		  [&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> &,
		      const bool &) {
			  ++callbackCount;
		  });

		const std::vector<std::uint8_t> malformedData = {
			0x80,
			0x7D,
			0x00,
			0x00,
			0x00,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> validLengthData = {
			0x80,
			0x7D,
			0x00,
			0x00,
			0x00,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               malformedData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               validLengthData,
		               nullptr));

		EXPECT_EQ(0U, callbackCount);
		EXPECT_EQ(0U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(nullptr, interfaceUnderTest.get_received_guidance_machine_info(0));
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, UnhandledParameterGroupNumberHasNoEffect)
	{
		TestableAgriculturalGuidanceInterface interfaceUnderTest;

		std::size_t machineInfoCallbackCount = 0;
		std::size_t systemCommandCallbackCount = 0;

		interfaceUnderTest.get_guidance_machine_info_event_publisher().add_listener(
		  [&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceMachineInfo> &,
		      const bool &) {
			  ++machineInfoCallbackCount;
		  });

		interfaceUnderTest.get_guidance_system_command_event_publisher().add_listener(
		  [&](const std::shared_ptr<AgriculturalGuidanceInterface::GuidanceSystemCommand> &,
		      const bool &) {
			  ++systemCommandCallbackCount;
		  });

		const std::vector<std::uint8_t> data = {
			0x80,
			0x7D,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::LanguageCommand,
		               data,
		               sourceOne));

		EXPECT_EQ(0U, machineInfoCallbackCount);
		EXPECT_EQ(0U, systemCommandCallbackCount);
		EXPECT_EQ(0U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(0U,
		          interfaceUnderTest.get_number_received_guidance_system_command_sources());
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, ReceivedMessagesExpireAtExactTimeout)
	{
		TestableAgriculturalGuidanceInterface interfaceUnderTest;
		interfaceUnderTest.initialize();

		const std::vector<std::uint8_t> machineInfoData = {
			0x80,
			0x7D,
			0x00,
			0x00,
			0x00,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> systemCommandData = {
			0x80,
			0x7D,
			0xFC,
			0xFF,
			0xFF,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               machineInfoData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceSystemCommand,
		               systemCommandData,
		               sourceTwo));

		ASSERT_EQ(1U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		ASSERT_EQ(1U,
		          interfaceUnderTest.get_number_received_guidance_system_command_sources());

		timeSource.set_time_ms(1149);
		interfaceUnderTest.update();

		EXPECT_EQ(1U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(1U,
		          interfaceUnderTest.get_number_received_guidance_system_command_sources());

		timeSource.set_time_ms(1150);
		interfaceUnderTest.update();

		EXPECT_EQ(0U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());
		EXPECT_EQ(0U,
		          interfaceUnderTest.get_number_received_guidance_system_command_sources());
		EXPECT_EQ(nullptr, interfaceUnderTest.get_received_guidance_machine_info(0));
		EXPECT_EQ(nullptr, interfaceUnderTest.get_received_guidance_system_command(0));
	}

	TEST_F(AgriculturalGuidanceGeneratedTest, UpdatingOneSourceRefreshesOnlyThatSourceTimestamp)
	{
		TestableAgriculturalGuidanceInterface interfaceUnderTest;
		interfaceUnderTest.initialize();

		const std::vector<std::uint8_t> sourceOneData = {
			0x80,
			0x7D,
			0x00,
			0x00,
			0x00,
			0xFF,
			0xFF,
			0xFF
		};

		const std::vector<std::uint8_t> sourceTwoData = {
			0x84,
			0x7D,
			0x55,
			0x20,
			0x41,
			0xFF,
			0xFF,
			0xFF
		};

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               sourceOneData,
		               sourceOne));

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               sourceTwoData,
		               sourceTwo));

		timeSource.set_time_ms(1100);

		interfaceUnderTest.process_received_message(
		  make_message(CANLibParameterGroupNumber::AgriculturalGuidanceMachineInfo,
		               sourceTwoData,
		               sourceTwo));

		timeSource.set_time_ms(1150);
		interfaceUnderTest.update();

		ASSERT_EQ(1U,
		          interfaceUnderTest.get_number_received_guidance_machine_info_message_sources());

		const auto remainingInfo =
		  interfaceUnderTest.get_received_guidance_machine_info(0);

		ASSERT_NE(nullptr, remainingInfo);
		EXPECT_EQ(sourceTwo, remainingInfo->get_sender_control_function());
		EXPECT_FLOAT_EQ(1.0F, remainingInfo->get_estimated_curvature());
		EXPECT_EQ(1100U, remainingInfo->get_timestamp_ms());
	}
} // namespace