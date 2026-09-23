#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>
#include <cstring>

#include "core/VirtualClock.h"
#include "core/SimulatedGPIO.h"
#include "core/FiberScheduler.h"
#include "core/BusRegistry.h"
#include "bus/CANBus.h"
#include "hal_mock/CAN.h"
#include "peripherals/ICANDevice.h"
#include "peripherals/MksServo57D_Sim.h"
#include "peripherals/TVCActuator_Sim.h"

// Production firmware headers
#include "can_bus/toad_can_bus.h"
#include "hardware_mapping/ec_pins.h"

using namespace toad::sim;

void test_can_frame_data_structures() {
    std::cout << "[Test 1] Testing CanFrame data structure, packing, and equality..." << std::endl;

    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    CanFrame f1(0x123, payload, sizeof(payload), false);

    assert(f1.id == 0x123);
    assert(!f1.extended);
    assert(!f1.rtr);
    assert(f1.len == 5);
    assert(std::memcmp(f1.data, payload, 5) == 0);

    CanFrame f2 = f1;
    assert(f1 == f2);

    f2.data[0] = 0xFF;
    assert(f1 != f2);

    // Extended frame
    CanFrame f_ext(0x18DAF110, payload, sizeof(payload), true);
    assert(f_ext.extended);
    assert(f_ext.id == 0x18DAF110);

    std::cout << "  -> PASSED" << std::endl;
}

void test_firmware_to_bus_transmit() {
    std::cout << "[Test 2] Testing Firmware CAN (HardwareCAN) transmit and read..." << std::endl;
    VirtualClock::instance().reset(0);
    BusRegistry::instance().reset();

    CAN can_controller("CAN_TVC");

    // Test HardwareCAN polymorphism and CanBitRate enum
    arduino::HardwareCAN* hw_can = &can_controller;
    bool begin_ok = hw_can->begin(arduino::CanBitRate::BR_500k);
    assert(begin_ok);

    auto bus = can_controller.get_bus();
    assert(bus != nullptr);
    bus->set_zero_latency(true);

    uint8_t tx_data[] = {0xAA, 0xBB, 0xCC};
    arduino::CanMsg tx_msg(0x100, sizeof(tx_data), tx_data);
    int write_result = hw_can->write(tx_msg);
    assert(write_result == 1);

    // Since it was sent by firmware without external nodes, nothing in firmware RX yet
    assert(hw_can->available() == 0);

    // Now inject frame as if from external bus node
    CanFrame incoming(0x200, tx_data, sizeof(tx_data));
    auto ext_dev = std::make_shared<FunctionalCANDevice>("ExtNode", 0x200);
    bus->broadcast(incoming, ext_dev.get()); // Real node sender = external peripheral

    assert(hw_can->available() == 1);
    arduino::CanMsg rcv = hw_can->read();
    assert(rcv.getStandardId() == 0x200);
    assert(rcv.data_length == 3);
    assert(rcv.data[0] == 0xAA && rcv.data[1] == 0xBB && rcv.data[2] == 0xCC);

    std::cout << "  -> PASSED" << std::endl;
}


void test_multi_drop_broadcast_delivery() {
    std::cout << "[Test 3] Testing CANBus multi-drop broadcast delivery to multiple nodes..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("TEST_BUS", 500000);
    bus->set_zero_latency(true);

    int dev1_count = 0;
    int dev2_count = 0;

    auto dev1 = std::make_shared<FunctionalCANDevice>("Node1", 0x100,
        [&](const CanFrame&, FunctionalCANDevice&) { dev1_count++; }, 0x000); // 0x000 mask = accept all

    auto dev2 = std::make_shared<FunctionalCANDevice>("Node2", 0x200,
        [&](const CanFrame&, FunctionalCANDevice&) { dev2_count++; }, 0x000);

    bus->subscribe(dev1);
    bus->subscribe(dev2);

    assert(bus->subscribers().size() == 2);

    // Broadcast frame from MCU
    uint8_t msg[] = {0x11, 0x22};
    CanFrame f(0x300, msg, 2);
    bus->broadcast(f, nullptr);

    assert(dev1_count == 1);
    assert(dev2_count == 1);

    // Broadcast from dev1: dev2 should receive, dev1 should not echo to itself
    CanFrame f_from_dev1(0x101, msg, 2);
    bus->broadcast(f_from_dev1, dev1.get());

    assert(dev1_count == 1); // unchanged
    assert(dev2_count == 2); // received

    std::cout << "  -> PASSED" << std::endl;
}

void test_hardware_id_acceptance_filtering() {
    std::cout << "[Test 4] Testing CAN hardware ID acceptance filtering..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("FILTER_BUS", 500000);
    bus->set_zero_latency(true);

    std::vector<uint32_t> dev_pitch_ids;
    std::vector<uint32_t> dev_yaw_ids;

    // Pitch device only accepts CAN_ID_TVC_PITCH (0x001)
    auto dev_pitch = std::make_shared<FunctionalCANDevice>("Pitch", CAN_ID_TVC_PITCH,
        [&](const CanFrame& f, FunctionalCANDevice&) { dev_pitch_ids.push_back(f.id); }, 0x7FF);

    // Yaw device only accepts CAN_ID_TVC_YAW (0x002)
    auto dev_yaw = std::make_shared<FunctionalCANDevice>("Yaw", CAN_ID_TVC_YAW,
        [&](const CanFrame& f, FunctionalCANDevice&) { dev_yaw_ids.push_back(f.id); }, 0x7FF);

    bus->subscribe(dev_pitch);
    bus->subscribe(dev_yaw);

    uint8_t data[] = {0x00};
    bus->broadcast(CanFrame(CAN_ID_TVC_PITCH, data, 1));
    bus->broadcast(CanFrame(CAN_ID_TVC_YAW, data, 1));
    bus->broadcast(CanFrame(CAN_ID_STEPPER_OX, data, 1)); // Neither should accept

    assert(dev_pitch_ids.size() == 1);
    assert(dev_pitch_ids[0] == CAN_ID_TVC_PITCH);

    assert(dev_yaw_ids.size() == 1);
    assert(dev_yaw_ids[0] == CAN_ID_TVC_YAW);

    std::cout << "  -> PASSED" << std::endl;
}

void test_can_virtual_clock_timing() {
    std::cout << "[Test 5] Testing CAN transmission timing in VirtualClock..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("TIMED_BUS", 500000); // 500 kbps
    bus->set_zero_latency(false);

    uint8_t data[8] = {0};
    CanFrame frame(0x123, data, 8); // 8-byte standard frame

    uint64_t expected_us = bus->calculate_transmission_time_us(frame);
    assert(expected_us > 200 && expected_us < 350); // ~111 bits + stuff bits at 2 us/bit ≈ 250 us

    uint64_t t_start = 0;
    uint64_t t_end = 0;

    auto f = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        t_start = VirtualClock::instance().now_us();
        bus->broadcast(frame, nullptr);
        t_end = VirtualClock::instance().now_us();
    });

    VirtualClock::instance().run_until(500);
    f.join();

    assert(t_end - t_start == expected_us);
    std::cout << "  Transmitted 8-byte frame at 500 kbps in " << (t_end - t_start) << " virtual microseconds." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}


class TestCanObserver : public ICanObserver {
public:
    void on_can_transaction(const CanTransaction& tx) override {
        transactions.push_back(tx);
    }
    std::vector<CanTransaction> transactions;
};

void test_can_transaction_observable() {
    std::cout << "[Test 6] Testing CAN transaction observer & history..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("OBSERVED_BUS", 500000);
    bus->set_zero_latency(true);

    auto observer = std::make_shared<TestCanObserver>();
    bus->add_observer(observer);

    uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    CanFrame f(0x555, payload, 4);
    bus->broadcast(f, nullptr);

    assert(observer->transactions.size() == 1);
    assert(observer->transactions[0].can_id == 0x555);
    assert(observer->transactions[0].data.size() == 4);
    assert(observer->transactions[0].data[0] == 0xDE);

    assert(bus->transaction_history().size() == 1);
    auto bus_txs = bus->get_bus_transactions();
    assert(bus_txs.size() == 1);
    assert(bus_txs[0].channel_or_id == 0x555);

    std::cout << "  -> PASSED" << std::endl;
}

void test_functional_can_device_concurrency() {
    std::cout << "[Test 7] Testing FunctionalCANDevice with independent background fiber..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("CONCURRENT_BUS", 500000);
    bus->set_zero_latency(true);

    bool handler_called = false;
    uint32_t received_id = 0;

    auto dev = std::make_shared<FunctionalCANDevice>("ConcurrentResponder", 0x120,
        [&](const CanFrame& f, FunctionalCANDevice& d) {
            handler_called = true;
            received_id = f.id;
            // Send reply frame back
            uint8_t reply_data[] = {0xCA, 0xFE};
            CanFrame reply(0x121, reply_data, 2);
            d.transmit(reply);
        });

    bus->subscribe(dev);
    dev->start(PRIO_SENSORS);
    assert(dev->is_running());

    bool fw_completed = false;
    CanFrame fw_rcv;

    auto f_fw = launch_fiber_with_priority(PRIO_FIRMWARE, [&]() {
        uint8_t req[] = {0x01};
        bus->transmit_from_firmware(CanFrame(0x120, req, 1));

        // Yield/delay to let sensor fiber process
        delayMicroseconds(50);

        assert(bus->firmware_available() == 1);
        bus->read_to_firmware(fw_rcv);
        fw_completed = true;
    });

    VirtualClock::instance().run_until(1000);
    f_fw.join();
    dev->stop();
    assert(!dev->is_running());

    assert(fw_completed);
    assert(handler_called);
    assert(received_id == 0x120);
    assert(fw_rcv.id == 0x121);
    assert(fw_rcv.len == 2);
    assert(fw_rcv.data[0] == 0xCA && fw_rcv.data[1] == 0xFE);

    std::cout << "  -> PASSED" << std::endl;
}

void test_mks_servo_57d_sim_physics_concurrency() {
    std::cout << "[Test 8] Testing MksServo57D_Sim physics integration & command decoding..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("MOTOR_BUS", 500000);
    bus->set_zero_latency(true);

    auto motor = std::make_shared<MksServo57D_Sim>("StepperOX", CAN_ID_STEPPER_OX);
    bus->subscribe(motor);

    motor->start(PRIO_ACTUATOR_PHYSICS);
    assert(motor->is_running());

    // 1. Send speed command: 0xF6, forward, 300 RPM, acc = 32
    // byte 0: 0xF6
    // byte 1: 0x81 (forward dir bit 7 = 1, upper speed bits = 1)
    // byte 2: 0x2C (300 = 0x012C -> byte 1 upper = 1, byte 2 lower = 0x2C)
    // byte 3: 32 (acceleration)
    // byte 4: CRC
    uint8_t cmd_data[4] = {0xF6, 0x81, 0x2C, 32};
    uint8_t crc = MksServo57D_Sim::calculate_checksum(CAN_ID_STEPPER_OX, cmd_data, 4);

    uint8_t full_frame[5] = {cmd_data[0], cmd_data[1], cmd_data[2], cmd_data[3], crc};
    bus->broadcast(CanFrame(CAN_ID_STEPPER_OX, full_frame, 5), nullptr);

    // Run simulation for 100 ms (100,000 us) to let velocity ramp and shaft rotate
    VirtualClock::instance().run_until(100000);

    assert(motor->command_count() == 1);
    assert(motor->target_speed_rpm() == 300);
    assert(motor->current_speed_rpm() > 200.0f); // Has ramped up significantly
    assert(motor->current_angle_deg() > 10.0f);   // Has turned significantly
    std::cout << "  Motor speed: " << motor->current_speed_rpm() << " RPM, Shaft angle: "
              << motor->current_angle_deg() << " deg." << std::endl;

    // 2. Test CRC error rejection
    uint8_t bad_frame[5] = {0xF6, 0x00, 0x00, 32, 0x00}; // intentionally bad CRC
    bus->broadcast(CanFrame(CAN_ID_STEPPER_OX, bad_frame, 5), nullptr);
    VirtualClock::instance().run_until(105000);

    assert(motor->crc_error_count() == 1);
    assert(motor->command_count() == 1); // Not incremented

    // 3. Test telemetry request (0x30)
    uint8_t query_data[2] = {0x30, static_cast<uint8_t>(CAN_ID_STEPPER_OX + 0x30)};
    bus->broadcast(CanFrame(CAN_ID_STEPPER_OX, query_data, 2), nullptr);
    VirtualClock::instance().run_until(110000);

    assert(bus->firmware_available() == 1);
    CanFrame telem;
    bus->read_to_firmware(telem);
    assert(telem.id == CAN_ID_STEPPER_OX);
    assert(telem.len == 6);
    assert(telem.data[0] == 0x31); // Telemetry response

    motor->stop();
    assert(!motor->is_running());
    std::cout << "  -> PASSED" << std::endl;
}

void test_tvc_actuator_sim_concurrency() {
    std::cout << "[Test 9] Testing TVCActuator_Sim linear stroke physics..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("TVC_BUS", 500000);
    bus->set_zero_latency(true);

    auto actuator = std::make_shared<TVCActuator_Sim>("PitchActuator", CAN_ID_TVC_PITCH, 50.0f);
    bus->subscribe(actuator);

    actuator->start(PRIO_ACTUATOR_PHYSICS);
    assert(actuator->is_running());
    assert(actuator->current_length_mm() == 50.0f);

    // Send length setpoint: 75.0 mm -> fixed 750 (0x02EE)
    // byte 0: 0x20
    // byte 1: 0x02
    // byte 2: 0xEE
    uint8_t cmd[3] = {0x20, 0x02, 0xEE};
    bus->broadcast(CanFrame(CAN_ID_TVC_PITCH, cmd, 3), nullptr);

    // Advance 500 ms (500,000 us). At 25 mm/s, actuator extends ~12.5 mm -> length ~62.5 mm
    VirtualClock::instance().run_until(500000);

    assert(actuator->command_count() == 1);
    assert(actuator->target_length_mm() == 75.0f);
    assert(actuator->current_length_mm() > 60.0f && actuator->current_length_mm() < 65.0f);

    // Advance another 600 ms -> should reach setpoint 75.0 mm
    VirtualClock::instance().run_until(1100000);
    assert(std::abs(actuator->current_length_mm() - 75.0f) < 0.1f);

    actuator->stop();
    assert(!actuator->is_running());
    std::cout << "  Actuator extended to " << actuator->current_length_mm() << " mm." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

void test_toad_can_bus_decoder_integration() {
    std::cout << "[Test 10] Testing production toad_can_bus.h CAN_Msg_Decoder integration..." << std::endl;
    VirtualClock::instance().reset(0);

    enum test_state_t { STATE_INIT, STATE_RUNNING };
    test_state_t state = STATE_RUNNING;

    can_msg_heartbeat_t hb;
    hb.cmd_id = 0x00;

    CAN_Msg_Decoder<test_state_t> decoder(reinterpret_cast<const uint8_t*>(&hb), sizeof(hb), state);
    auto decoded_hb = decoder.decode<can_msg_heartbeat_t>();
    assert(decoded_hb.has_value());
    assert(decoded_hb->cmd_id == 0x00);

    std::cout << "  -> PASSED" << std::endl;
}

void test_multiple_concurrent_actuators_teardown() {
    std::cout << "[Test 11] Testing multiple concurrent actuators with clean teardown..." << std::endl;
    VirtualClock::instance().reset(0);

    auto bus = std::make_shared<CANBus>("FULL_CAN", 500000);
    bus->set_zero_latency(true);

    auto ox_motor = std::make_shared<MksServo57D_Sim>("OX_Motor", CAN_ID_STEPPER_OX);
    auto fu_motor = std::make_shared<MksServo57D_Sim>("FU_Motor", CAN_ID_STEPPER_FU);
    auto pitch_act = std::make_shared<TVCActuator_Sim>("Pitch_Act", CAN_ID_TVC_PITCH, 50.0f);
    auto yaw_act = std::make_shared<TVCActuator_Sim>("Yaw_Act", CAN_ID_TVC_YAW, 50.0f);

    bus->subscribe(ox_motor);
    bus->subscribe(fu_motor);
    bus->subscribe(pitch_act);
    bus->subscribe(yaw_act);

    ox_motor->start(PRIO_ACTUATOR_PHYSICS);
    fu_motor->start(PRIO_ACTUATOR_PHYSICS);
    pitch_act->start(PRIO_ACTUATOR_PHYSICS);
    yaw_act->start(PRIO_ACTUATOR_PHYSICS);

    assert(ox_motor->is_running() && fu_motor->is_running());
    assert(pitch_act->is_running() && yaw_act->is_running());

    VirtualClock::instance().run_until(50000);

    // Stop all actuators
    ox_motor->stop();
    fu_motor->stop();
    pitch_act->stop();
    yaw_act->stop();

    assert(!ox_motor->is_running() && !fu_motor->is_running());
    assert(!pitch_act->is_running() && !yaw_act->is_running());

    std::cout << "  All 4 concurrent peripheral fibers stopped cleanly without deadlock." << std::endl;
    std::cout << "  -> PASSED" << std::endl;
}

int main() {
    std::cout << "=== Running CANBus & Actuator Emulation Tests ===" << std::endl;
    install_fiber_scheduler();

    test_can_frame_data_structures();
    test_firmware_to_bus_transmit();
    test_multi_drop_broadcast_delivery();
    test_hardware_id_acceptance_filtering();
    test_can_virtual_clock_timing();
    test_can_transaction_observable();
    test_functional_can_device_concurrency();
    test_mks_servo_57d_sim_physics_concurrency();
    test_tvc_actuator_sim_concurrency();
    test_toad_can_bus_decoder_integration();
    test_multiple_concurrent_actuators_teardown();

    std::cout << "=== All CANBus & Actuator Emulation Tests Passed Successfully! ===" << std::endl;
    return 0;
}
