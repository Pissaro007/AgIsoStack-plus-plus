#include <gtest/gtest.h>

#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_identifier.hpp"
#include "isobus/isobus/can_message.hpp"
#include "isobus/isobus/isobus_maintain_power_interface.hpp"
#include "isobus/utility/system_timing.hpp"
#include "isobus/utility/time_source.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace
{
using isobus::CANIdentifier;
using isobus::CANLibParameterGroupNumber;
using isobus::CANMessage;
using isobus::MaintainPowerInterface;
using isobus::SystemTiming;
using isobus::TimeSource;

using MaintainPowerData = MaintainPowerInterface::MaintainPowerData;

class ManualTimeSource : public TimeSource
{
public:
	explicit ManualTimeSource(std::uint32_t initialTime_ms = 1) :
	  currentTime_ms(initialTime_ms)
	{
	}

	std::uint32_t get_current_time_ms() const override
	{
		return currentTime_ms;
	}

	std::uint64_t get_current_time_us() const override
	{
		return static_cast<std::uint64_t>(currentTime_ms) * 1000ULL;
	}

	void set_time_ms(std::uint32_t value)
	{
		currentTime_ms = value;
	}

	void advance_ms(std::uint32_t amount)
	{
		currentTime_ms += amount;
	}

private:
	std::uint32_t currentTime_ms;
};

class TimeSourceOverrideGuard
{
public:
	explicit TimeSourceOverrideGuard(TimeSource *source)
	{
		SystemTiming::override_time_source(source);
	}

	~TimeSourceOverrideGuard()
	{
		SystemTiming::override_time_source(nullptr);
	}

	TimeSourceOverrideGuard(const TimeSourceOverrideGuard &) = delete;
	TimeSourceOverrideGuard &operator=(const TimeSourceOverrideGuard &) = delete;
};

class TestableMaintainPowerInterface : public MaintainPowerInterface
{
public:
	explicit TestableMaintainPowerInterface(
	  std::shared_ptr<isobus::InternalControlFunction> sourceControlFunction) :
	  MaintainPowerInterface(sourceControlFunction)
	{
	}

	void process_received_message_for_test(const CANMessage &message)
	{
		process_rx_message(message, this);
	}
};

CANMessage make_receive_message(std::uint32_t parameterGroupNumber,
                                const std::vector<std::uint8_t> &data)
{
	const CANIdentifier identifier(
	  CANIdentifier::Type::Extended,
	  parameterGroupNumber,
	  CANIdentifier::CANPriority::PriorityDefault6,
	  CANIdentifier::GLOBAL_ADDRESS,
	  0x80);

	return CANMessage(
	  CANMessage::Type::Receive,
	  identifier,
	  data,
	  std::shared_ptr<isobus::ControlFunction>(),
	  std::shared_ptr<isobus::ControlFunction>(),
	  0);
}

TEST(MaintainPowerDataTests, HasDocumentedInitialState)
{
	MaintainPowerData data(nullptr);

	EXPECT_EQ(nullptr, data.get_sender_control_function());
	EXPECT_EQ(0U, data.get_timestamp_ms());

	EXPECT_EQ(
	  MaintainPowerData::ImplementInWorkState::NotAvailable,
	  data.get_implement_in_work_state());

	EXPECT_EQ(
	  MaintainPowerData::ImplementReadyToWorkState::NotAvailable,
	  data.get_implement_ready_to_work_state());

	EXPECT_EQ(
	  MaintainPowerData::ImplementParkState::NotAvailable,
	  data.get_implement_park_state());

	EXPECT_EQ(
	  MaintainPowerData::ImplementTransportState::NotAvailable,
	  data.get_implement_transport_state());

	EXPECT_EQ(
	  MaintainPowerData::MaintainActuatorPower::DontCare,
	  data.get_maintain_actuator_power());

	EXPECT_EQ(
	  MaintainPowerData::MaintainECUPower::DontCare,
	  data.get_maintain_ecu_power());
}

TEST(MaintainPowerDataTests, StoresTimestamp)
{
	MaintainPowerData data(nullptr);

	data.set_timestamp_ms(123456U);

	EXPECT_EQ(123456U, data.get_timestamp_ms());
}

TEST(MaintainPowerDataTests, ImplementInWorkSetterReportsChanges)
{
	MaintainPowerData data(nullptr);

	EXPECT_FALSE(data.set_implement_in_work_state(
	  MaintainPowerData::ImplementInWorkState::NotAvailable));

	EXPECT_TRUE(data.set_implement_in_work_state(
	  MaintainPowerData::ImplementInWorkState::ImplementInWorkState));

	EXPECT_EQ(
	  MaintainPowerData::ImplementInWorkState::ImplementInWorkState,
	  data.get_implement_in_work_state());

	EXPECT_FALSE(data.set_implement_in_work_state(
	  MaintainPowerData::ImplementInWorkState::ImplementInWorkState));
}

TEST(MaintainPowerDataTests, ImplementReadyToWorkSetterReportsChanges)
{
	MaintainPowerData data(nullptr);

	EXPECT_FALSE(data.set_implement_ready_to_work_state(
	  MaintainPowerData::ImplementReadyToWorkState::NotAvailable));

	EXPECT_TRUE(data.set_implement_ready_to_work_state(
	  MaintainPowerData::ImplementReadyToWorkState::
	    ImplementReadyForFieldWork));

	EXPECT_EQ(
	  MaintainPowerData::ImplementReadyToWorkState::
	    ImplementReadyForFieldWork,
	  data.get_implement_ready_to_work_state());

	EXPECT_FALSE(data.set_implement_ready_to_work_state(
	  MaintainPowerData::ImplementReadyToWorkState::
	    ImplementReadyForFieldWork));
}

TEST(MaintainPowerDataTests, ImplementParkSetterReportsChanges)
{
	MaintainPowerData data(nullptr);

	EXPECT_FALSE(data.set_implement_park_state(
	  MaintainPowerData::ImplementParkState::NotAvailable));

	EXPECT_TRUE(data.set_implement_park_state(
	  MaintainPowerData::ImplementParkState::ImplementMayBeDisconnected));

	EXPECT_EQ(
	  MaintainPowerData::ImplementParkState::ImplementMayBeDisconnected,
	  data.get_implement_park_state());

	EXPECT_FALSE(data.set_implement_park_state(
	  MaintainPowerData::ImplementParkState::ImplementMayBeDisconnected));
}

TEST(MaintainPowerDataTests, ImplementTransportSetterReportsChanges)
{
	MaintainPowerData data(nullptr);

	EXPECT_FALSE(data.set_implement_transport_state(
	  MaintainPowerData::ImplementTransportState::NotAvailable));

	EXPECT_TRUE(data.set_implement_transport_state(
	  MaintainPowerData::ImplementTransportState::
	    ImplementMayBeTransported));

	EXPECT_EQ(
	  MaintainPowerData::ImplementTransportState::
	    ImplementMayBeTransported,
	  data.get_implement_transport_state());

	EXPECT_FALSE(data.set_implement_transport_state(
	  MaintainPowerData::ImplementTransportState::
	    ImplementMayBeTransported));
}

TEST(MaintainPowerDataTests, MaintainActuatorPowerSetterReportsChanges)
{
	MaintainPowerData data(nullptr);

	EXPECT_FALSE(data.set_maintain_actuator_power(
	  MaintainPowerData::MaintainActuatorPower::DontCare));

	EXPECT_TRUE(data.set_maintain_actuator_power(
	  MaintainPowerData::MaintainActuatorPower::
	    RequirementFor2SecondsMoreForPWR));

	EXPECT_EQ(
	  MaintainPowerData::MaintainActuatorPower::
	    RequirementFor2SecondsMoreForPWR,
	  data.get_maintain_actuator_power());

	EXPECT_FALSE(data.set_maintain_actuator_power(
	  MaintainPowerData::MaintainActuatorPower::
	    RequirementFor2SecondsMoreForPWR));
}

TEST(MaintainPowerDataTests, MaintainECUPowerSetterReportsChanges)
{
	MaintainPowerData data(nullptr);

	EXPECT_FALSE(data.set_maintain_ecu_power(
	  MaintainPowerData::MaintainECUPower::DontCare));

	EXPECT_TRUE(data.set_maintain_ecu_power(
	  MaintainPowerData::MaintainECUPower::
	    RequirementFor2SecondsMoreForECU_PWR));

	EXPECT_EQ(
	  MaintainPowerData::MaintainECUPower::
	    RequirementFor2SecondsMoreForECU_PWR,
	  data.get_maintain_ecu_power());

	EXPECT_FALSE(data.set_maintain_ecu_power(
	  MaintainPowerData::MaintainECUPower::
	    RequirementFor2SecondsMoreForECU_PWR));
}

TEST(MaintainPowerDataTests, SettersAcceptAllDeclaredStates)
{
	MaintainPowerData data(nullptr);

	EXPECT_TRUE(data.set_implement_in_work_state(
	  MaintainPowerData::ImplementInWorkState::ErrorIndication));

	EXPECT_TRUE(data.set_implement_ready_to_work_state(
	  MaintainPowerData::ImplementReadyToWorkState::ErrorIndication));

	EXPECT_TRUE(data.set_implement_park_state(
	  MaintainPowerData::ImplementParkState::ErrorIndication));

	EXPECT_TRUE(data.set_implement_transport_state(
	  MaintainPowerData::ImplementTransportState::ErrorIndication));

	EXPECT_TRUE(data.set_maintain_actuator_power(
	  MaintainPowerData::MaintainActuatorPower::Reserved));

	EXPECT_TRUE(data.set_maintain_ecu_power(
	  MaintainPowerData::MaintainECUPower::Reserved));

	EXPECT_EQ(
	  MaintainPowerData::ImplementInWorkState::ErrorIndication,
	  data.get_implement_in_work_state());

	EXPECT_EQ(
	  MaintainPowerData::ImplementReadyToWorkState::ErrorIndication,
	  data.get_implement_ready_to_work_state());

	EXPECT_EQ(
	  MaintainPowerData::ImplementParkState::ErrorIndication,
	  data.get_implement_park_state());

	EXPECT_EQ(
	  MaintainPowerData::ImplementTransportState::ErrorIndication,
	  data.get_implement_transport_state());

	EXPECT_EQ(
	  MaintainPowerData::MaintainActuatorPower::Reserved,
	  data.get_maintain_actuator_power());

	EXPECT_EQ(
	  MaintainPowerData::MaintainECUPower::Reserved,
	  data.get_maintain_ecu_power());
}

TEST(MaintainPowerInterfaceTests, StartsUninitializedAndWithoutReceivedData)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);

	EXPECT_FALSE(interfaceUnderTest.get_initialized());
	EXPECT_EQ(0U,
	          interfaceUnderTest.get_number_received_maintain_power_sources());
	EXPECT_EQ(nullptr,
	          interfaceUnderTest.get_received_maintain_power(0));
}

TEST(MaintainPowerInterfaceTests, InitializeSetsInitializedState)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);

	interfaceUnderTest.initialize();

	EXPECT_TRUE(interfaceUnderTest.get_initialized());
}

TEST(MaintainPowerInterfaceTests, InitializeIsPubliclyIdempotent)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);

	interfaceUnderTest.initialize();
	interfaceUnderTest.initialize();

	EXPECT_TRUE(interfaceUnderTest.get_initialized());
	EXPECT_EQ(0U,
	          interfaceUnderTest.get_number_received_maintain_power_sources());
}

TEST(MaintainPowerInterfaceTests, StoresConfiguredMaintainPowerTime)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);

	EXPECT_EQ(0U, interfaceUnderTest.get_maintain_power_time());

	interfaceUnderTest.set_maintain_power_time(4500U);
	EXPECT_EQ(4500U, interfaceUnderTest.get_maintain_power_time());

	interfaceUnderTest.set_maintain_power_time(0U);
	EXPECT_EQ(0U, interfaceUnderTest.get_maintain_power_time());
}

TEST(MaintainPowerInterfaceTests, OutOfRangeReceivedMessageIndexReturnsNull)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);

	EXPECT_EQ(nullptr,
	          interfaceUnderTest.get_received_maintain_power(0));
	EXPECT_EQ(nullptr,
	          interfaceUnderTest.get_received_maintain_power(1));
	EXPECT_EQ(nullptr,
	          interfaceUnderTest.get_received_maintain_power(1000));
}

TEST(MaintainPowerInterfaceTests,
     MalformedMaintainPowerMessageIsIgnoredWithoutNotification)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);
	std::size_t notificationCount = 0;

	const isobus::EventCallbackHandle listener =
	  interfaceUnderTest.get_maintain_power_data_event_publisher().add_listener(
	    [&notificationCount]<MaintainPowerData> &,
	      const bool & {
		    ++notificationCount;
	    });

	const std::vector<std::uint8_t> malformedData(7, 0xFF);
	const CANMessage malformedMessage = make_receive_message(
	  static_cast<std::uint32_t>(
	    CANLibParameterGroupNumber::MaintainPower),
	  malformedData);

	interfaceUnderTest.process_received_message_for_test(malformedMessage);

	EXPECT_EQ(0U, notificationCount);
	EXPECT_EQ(0U,
	          interfaceUnderTest.get_number_received_maintain_power_sources());

	interfaceUnderTest.get_maintain_power_data_event_publisher()
	  .remove_listener(listener);
}

TEST(MaintainPowerInterfaceTests,
     EightByteMaintainPowerMessageWithoutSourceIsIgnored)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);
	std::size_t notificationCount = 0;

	const isobus::EventCallbackHandle listener =
	  interfaceUnderTest.get_maintain_power_data_event_publisher().add_listener(
	   [&notificationCount]<MaintainPowerData> &,
	      const bool & {
		    ++notificationCount;
	    });

	std::vector<std::uint8_t> data(8, 0xFF);
	data[0] = 0x5F;
	data[1] = 0x55;

	const CANMessage messageWithoutSource = make_receive_message(
	  static_cast<std::uint32_t>(
	    CANLibParameterGroupNumber::MaintainPower),
	  data);

	interfaceUnderTest.process_received_message_for_test(
	  messageWithoutSource);

	EXPECT_EQ(0U, notificationCount);
	EXPECT_EQ(0U,
	          interfaceUnderTest.get_number_received_maintain_power_sources());

	interfaceUnderTest.get_maintain_power_data_event_publisher()
	  .remove_listener(listener);
}

TEST(MaintainPowerInterfaceTests,
     MalformedWheelBasedSpeedMessageDoesNotPublishKeyOffTransition)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);
	std::size_t transitionCount = 0;

	const isobus::EventCallbackHandle listener =
	  interfaceUnderTest.get_key_switch_transition_off_event_publisher()
	    .add_listener([&transitionCount]() {
		    ++transitionCount;
	    });

	const std::vector<std::uint8_t> malformedData(7, 0x00);
	const CANMessage malformedMessage = make_receive_message(
	  static_cast<std::uint32_t>(
	    CANLibParameterGroupNumber::WheelBasedSpeedAndDistance),
	  malformedData);

	interfaceUnderTest.process_received_message_for_test(malformedMessage);

	EXPECT_EQ(0U, transitionCount);

	interfaceUnderTest.get_key_switch_transition_off_event_publisher()
	  .remove_listener(listener);
}

TEST(MaintainPowerInterfaceTests,
     EightByteWheelBasedSpeedMessageWithoutSourceIsIgnored)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);
	std::size_t transitionCount = 0;

	const isobus::EventCallbackHandle listener =
	  interfaceUnderTest.get_key_switch_transition_off_event_publisher()
	    .add_listener([&transitionCount]() {
		    ++transitionCount;
	    });

	std::vector<std::uint8_t> data(8, 0x00);
	data[7] = 0x04;

	const CANMessage messageWithoutSource = make_receive_message(
	  static_cast<std::uint32_t>(
	    CANLibParameterGroupNumber::WheelBasedSpeedAndDistance),
	  data);

	interfaceUnderTest.process_received_message_for_test(
	  messageWithoutSource);

	EXPECT_EQ(0U, transitionCount);

	interfaceUnderTest.get_key_switch_transition_off_event_publisher()
	  .remove_listener(listener);
}

TEST(MaintainPowerInterfaceTests,
     UpdateWithManualTimeAndNoReceivedMessagesIsSafe)
{
	ManualTimeSource timeSource(100U);
	TimeSourceOverrideGuard timeGuard(&timeSource);
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);

	interfaceUnderTest.initialize();

	EXPECT_NO_THROW(interfaceUnderTest.update());

	timeSource.advance_ms(2500U);

	EXPECT_NO_THROW(interfaceUnderTest.update());
	EXPECT_EQ(0U,
	          interfaceUnderTest.get_number_received_maintain_power_sources());
}

TEST(MaintainPowerInterfaceTests,
     EventListenersCanBeRemovedWithoutLeakingIntoLaterOperations)
{
	TestableMaintainPowerInterface interfaceUnderTest(nullptr);
	std::size_t notificationCount = 0;

	const isobus::EventCallbackHandle listener =
	  interfaceUnderTest.get_maintain_power_data_event_publisher().add_listener(
	    [&notificationCount]<MaintainPowerData> &,
	      const bool & {
		    ++notificationCount;
	    });

	interfaceUnderTest.get_maintain_power_data_event_publisher()
	  .remove_listener(listener);

	const std::vector<std::uint8_t> malformedData(7, 0xFF);
	const CANMessage malformedMessage = make_receive_message(
	  static_cast<std::uint32_t>(
	    CANLibParameterGroupNumber::MaintainPower),
	  malformedData);

	interfaceUnderTest.process_received_message_for_test(malformedMessage);

	EXPECT_EQ(0U, notificationCount);
}
} // namespace
