#include <stdio.h>
#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "Arduino.h"
#include "pins_arduino.h"
#include "hardware_mapping/ec_pins.h"
#include "hardware_mapping/ec_sensors.h"
#include "can_bus/toad_can_bus.h"

#include "core/VirtualClock.h"
#include "core/FiberScheduler.h"
#include "core/BusRegistry.h"
#include "core/SimulatedGPIO.h"

#include "bus/SPIBus.h"
#include "bus/UartBus.h"
#include "bus/RS485Mux.h"
#include "bus/CANBus.h"
#include "bus/UartSnooper.h"
#include "hal_mock/CAN.h"
#include "bus/SPISnooper.h"

#include "peripherals/ADS131M02_Sim.h"
#include "peripherals/MAX31856_Sim.h"
#include "peripherals/AMT242AV_Sim.h"
#include "peripherals/MksServo57D_Sim.h"
#include "peripherals/TVCActuator_Sim.h"

#include "linux_wrap.h"

// Forward declaration of Arduino sketch entrypoints in ec_main.cpp
extern void setup();
extern void loop();

static std::atomic<bool> g_shutdown{false};

static void signal_handler(int sig) {
    (void)sig;
    g_shutdown.store(true);
}

int main(void)
{
    terminal_begin(); // do any necessary setup for our terminal

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "========================================" << std::endl;
    std::cout << "  Starting Toad Engine Controller SITL  " << std::endl;
    std::cout << "========================================" << std::endl;

    // 1. Install cooperative Boost.Fiber scheduler on the simulation thread
    toad::sim::install_fiber_scheduler();

    // =========================================================
    // UART & RS485 Busses
    // =========================================================

    // RS485_6 (pins PG9, PG14, DE: PG12)
    auto rs485_6_bus = std::make_shared<toad::sim::RS485Mux>(115200, "RS485_6", PIN_RS485_6_DE);
    auto rs485_6_snooper = std::make_shared<toad::sim::UartSnooper>(
        "RS485_6", toad::sim::SnoopFormat::FORMAT_HEX_DUMP
    );
    rs485_6_bus->add_observer(rs485_6_snooper);
    toad::sim::BusRegistry::instance().register_uart(PIN_RS485_6_RX, PIN_RS485_6_TX, rs485_6_bus);

    // RS485_2 (pins PD6, PD5, DE: PD4)
    auto rs485_2_bus = std::make_shared<toad::sim::RS485Mux>(115200, "RS485_2", PIN_RS485_2_DE);
    auto rs485_2_snooper = std::make_shared<toad::sim::UartSnooper>(
        "RS485_2", toad::sim::SnoopFormat::FORMAT_HEX_DUMP
    );
    rs485_2_bus->add_observer(rs485_2_snooper);
    toad::sim::BusRegistry::instance().register_uart(PIN_RS485_2_RX, PIN_RS485_2_TX, rs485_2_bus);

    // USB Comms Serial
    auto usb_serial_bus = std::make_shared<toad::sim::UartBus>(115200, "USB");
    auto usb_snooper = std::make_shared<toad::sim::UartSnooper>(
        "USB", toad::sim::SnoopFormat::FORMAT_ASCII, nullptr, true, /*enable_input=*/true
    );
    usb_serial_bus->add_observer(usb_snooper);
    toad::sim::BusRegistry::instance().register_uart(0, 0, usb_serial_bus);

    // Hardware Comms Serial (UART5: PB12/PB13 or PIN_HW_COMM_SERIAL_*)
    auto hw_comms_bus = std::make_shared<toad::sim::UartBus>(115200, "HW Comms");
    auto hw_comms_snooper = std::make_shared<toad::sim::UartSnooper>(
        "HW Comms", toad::sim::SnoopFormat::FORMAT_ASCII, nullptr, true, /*enable_input=*/true
    );
    hw_comms_bus->add_observer(hw_comms_snooper);
    toad::sim::BusRegistry::instance().register_uart(PIN_HW_COMM_SERIAL_RX, PIN_HW_COMM_SERIAL_TX, hw_comms_bus);

    // Hardware Comms Fallback Serial (UART3)
    auto hw_fallback_bus = std::make_shared<toad::sim::UartBus>(115200, "HW Comms Fallback");
    auto hw_fallback_snooper = std::make_shared<toad::sim::UartSnooper>(
        "HW Comms Fallback", toad::sim::SnoopFormat::FORMAT_ASCII
    );
    hw_fallback_bus->add_observer(hw_fallback_snooper);
    toad::sim::BusRegistry::instance().register_uart(PIN_HW_FALLBACK_SERIAL_RX, PIN_HW_FALLBACK_SERIAL_TX, hw_fallback_bus);

    // =========================================================
    // SPI Busses (ec_pins.h)
    // =========================================================

    // PT_TC_SPI_1 (MOSI: PD7, MISO: PB4, SCK: PG11)
    auto spi1_bus = std::make_shared<toad::sim::SPIBus>(
        "PT_TC_SPI_1",
        PIN_PT_TC_SPI_1_MOSI,
        PIN_PT_TC_SPI_1_MISO,
        PIN_PT_TC_SPI_1_SCK
    );
    spi1_bus->set_zero_latency(true);
    toad::sim::BusRegistry::instance().register_spi(
        PIN_PT_TC_SPI_1_MOSI, PIN_PT_TC_SPI_1_MISO, PIN_PT_TC_SPI_1_SCK, spi1_bus
    );
    toad::sim::BusRegistry::instance().register_named_spi("PT_TC_SPI_1", spi1_bus);

    auto spi1_snooper = std::make_shared<toad::sim::SPISnooper>("SPI1");
    spi1_bus->add_observer(spi1_snooper);

    // PT_TC_SPI_3 (MOSI: PC12, MISO: PC11, SCK: PC10)
    auto spi3_bus = std::make_shared<toad::sim::SPIBus>(
        "PT_TC_SPI_3",
        PIN_PT_TC_SPI_3_MOSI,
        PIN_PT_TC_SPI_3_MISO,
        PIN_PT_TC_SPI_3_SCK
    );
    spi3_bus->set_zero_latency(true);
    toad::sim::BusRegistry::instance().register_spi(
        PIN_PT_TC_SPI_3_MOSI, PIN_PT_TC_SPI_3_MISO, PIN_PT_TC_SPI_3_SCK, spi3_bus
    );
    toad::sim::BusRegistry::instance().register_named_spi("PT_TC_SPI_3", spi3_bus);

    auto spi3_snooper = std::make_shared<toad::sim::SPISnooper>("SPI3");
    spi3_bus->add_observer(spi3_snooper);

    // =========================================================
    // CAN Bus & Actuators
    // =========================================================
    CAN can_tvc("CAN_TVC");
    can_tvc.begin(arduino::CanBitRate::BR_500k);
    auto can_bus = can_tvc.get_bus();

    auto ox_motor = std::make_shared<toad::sim::MksServo57D_Sim>("OX_Motor", CAN_ID_STEPPER_OX);
    auto fu_motor = std::make_shared<toad::sim::MksServo57D_Sim>("FU_Motor", CAN_ID_STEPPER_FU);
    auto tvc_pitch = std::make_shared<toad::sim::TVCActuator_Sim>("TVC_Pitch", CAN_ID_TVC_PITCH);
    auto tvc_yaw = std::make_shared<toad::sim::TVCActuator_Sim>("TVC_Yaw", CAN_ID_TVC_YAW);

    if (can_bus) {
        can_bus->subscribe(ox_motor);
        can_bus->subscribe(fu_motor);
        can_bus->subscribe(tvc_pitch);
        can_bus->subscribe(tvc_yaw);
    }

    // =========================================================
    // Pressure Transducer (PT) ADCs (ADS131M02)
    // =========================================================
    auto pt_adc_1_2 = std::make_shared<toad::sim::ADS131M02_Sim>("PT_ADC_1_2");
    spi3_bus->register_device(PIN_PT_BOARD_1_2_CS, pt_adc_1_2);

    auto pt_adc_3_4 = std::make_shared<toad::sim::ADS131M02_Sim>("PT_ADC_3_4");
    spi3_bus->register_device(PIN_PT_BOARD_3_4_CS, pt_adc_3_4);

    auto pt_adc_5_6 = std::make_shared<toad::sim::ADS131M02_Sim>("PT_ADC_5_6");
    spi3_bus->register_device(PIN_PT_BOARD_5_6_CS, pt_adc_5_6);

    auto pt_adc_7_8 = std::make_shared<toad::sim::ADS131M02_Sim>("PT_ADC_7_8");
    spi3_bus->register_device(PIN_PT_BOARD_7_8_CS, pt_adc_7_8);

    auto pt_adc_9_10 = std::make_shared<toad::sim::ADS131M02_Sim>("PT_ADC_9_10");
    spi1_bus->register_device(PIN_PT_BOARD_9_10_CS, pt_adc_9_10);

    auto pt_adc_11_12 = std::make_shared<toad::sim::ADS131M02_Sim>("PT_ADC_11_12");
    spi1_bus->register_device(PIN_PT_BOARD_11_12_CS, pt_adc_11_12);

    // =========================================================
    // Thermocouple (TC) Chips (MAX31856)
    // =========================================================
    auto tc_chip_1 = std::make_shared<toad::sim::MAX31856_Sim>("TC_CHIP_1", 25.0f);
    spi3_bus->register_device(PIN_TC_CHIP_1_CS, tc_chip_1);

    auto tc_chip_2 = std::make_shared<toad::sim::MAX31856_Sim>("TC_CHIP_2", 25.0f);
    spi3_bus->register_device(PIN_TC_CHIP_2_CS, tc_chip_2);

    auto tc_chip_3 = std::make_shared<toad::sim::MAX31856_Sim>("TC_CHIP_3", 25.0f);
    spi1_bus->register_device(PIN_TC_CHIP_3_CS, tc_chip_3);

    auto tc_chip_4 = std::make_shared<toad::sim::MAX31856_Sim>("TC_CHIP_4", 25.0f);
    spi1_bus->register_device(PIN_TC_CHIP_4_CS, tc_chip_4);

    auto tc_chip_5 = std::make_shared<toad::sim::MAX31856_Sim>("TC_CHIP_5", 25.0f);
    spi1_bus->register_device(PIN_TC_CHIP_5_CS, tc_chip_5);

    auto tc_chip_6 = std::make_shared<toad::sim::MAX31856_Sim>("TC_CHIP_6", 25.0f);
    spi1_bus->register_device(PIN_TC_CHIP_6_CS, tc_chip_6);

    // =========================================================
    // RS485 Absolute Rotary Encoders (AMT242AV)
    // =========================================================
    auto enc_ox = std::make_shared<toad::sim::AMT242AV_Sim>(0x00, "ENC_OX");
    rs485_6_bus->register_device(PIN_ENC_OX_SEL, enc_ox);

    auto enc_fu = std::make_shared<toad::sim::AMT242AV_Sim>(0x00, "ENC_FU");
    rs485_2_bus->register_device(PIN_ENC_FU_SEL, enc_fu);

    // =========================================================
    // Start All Simulated Peripherals (Concurrent Background Fibers)
    // =========================================================
    std::cout << "[Simulation] Starting all simulated peripheral fibers..." << std::endl;

    pt_adc_1_2->start(toad::sim::PRIO_SENSORS);
    pt_adc_3_4->start(toad::sim::PRIO_SENSORS);
    pt_adc_5_6->start(toad::sim::PRIO_SENSORS);
    pt_adc_7_8->start(toad::sim::PRIO_SENSORS);
    pt_adc_9_10->start(toad::sim::PRIO_SENSORS);
    pt_adc_11_12->start(toad::sim::PRIO_SENSORS);

    tc_chip_1->start(toad::sim::PRIO_SENSORS);
    tc_chip_2->start(toad::sim::PRIO_SENSORS);
    tc_chip_3->start(toad::sim::PRIO_SENSORS);
    tc_chip_4->start(toad::sim::PRIO_SENSORS);
    tc_chip_5->start(toad::sim::PRIO_SENSORS);
    tc_chip_6->start(toad::sim::PRIO_SENSORS);

    enc_ox->start(toad::sim::PRIO_SENSORS);
    enc_fu->start(toad::sim::PRIO_SENSORS);

    ox_motor->start(toad::sim::PRIO_ACTUATOR_PHYSICS);
    fu_motor->start(toad::sim::PRIO_ACTUATOR_PHYSICS);
    tvc_pitch->start(toad::sim::PRIO_ACTUATOR_PHYSICS);
    tvc_yaw->start(toad::sim::PRIO_ACTUATOR_PHYSICS);

    // =========================================================
    // Launch Firmware Sketch on Independent Fiber (PRIO_FIRMWARE)
    // =========================================================
    std::cout << "[Simulation] Launching Arduino sketch fiber (PRIO_FIRMWARE = "
              << toad::sim::PRIO_FIRMWARE << ")..." << std::endl;

    auto fw_fiber = toad::sim::launch_fiber_with_priority(toad::sim::PRIO_FIRMWARE, []() {
        std::cout << "[Firmware Fiber] Calling setup()..." << std::endl;
        setup();
        std::cout << "[Firmware Fiber] setup() complete. Entering loop()..." << std::endl;

        while (!g_shutdown.load()) {
            loop();
            VirtualClock::instance().sleep_for(20); // 20 us virtual yield / sleep
        }
        std::cout << "[Firmware Fiber] loop() terminated." << std::endl;
    });

    // =========================================================
    // Main Thread: Paced Virtual Clock Progression
    // =========================================================
    std::cout << "[Simulation] Simulation running. Advancing virtual clock in real-time pace. Press Ctrl+C to exit." << std::endl;

    auto last_wall = std::chrono::steady_clock::now();
    while (!g_shutdown.load()) {
        auto now_wall = std::chrono::steady_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now_wall - last_wall).count();
        last_wall = now_wall;

        if (elapsed_us > 0) {
            VirtualClock::instance().advance_time_to(
                VirtualClock::instance().now_us() + elapsed_us
            );
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\n[Simulation] Shutdown signal received. Stopping peripherals..." << std::endl;

    // Wake all sleeping fibers to let them terminate
    VirtualClock::instance().wake_all();

    pt_adc_1_2->stop();
    pt_adc_3_4->stop();
    pt_adc_5_6->stop();
    pt_adc_7_8->stop();
    pt_adc_9_10->stop();
    pt_adc_11_12->stop();

    tc_chip_1->stop();
    tc_chip_2->stop();
    tc_chip_3->stop();
    tc_chip_4->stop();
    tc_chip_5->stop();
    tc_chip_6->stop();

    enc_ox->stop();
    enc_fu->stop();

    ox_motor->stop();
    fu_motor->stop();
    tvc_pitch->stop();
    tvc_yaw->stop();

    if (fw_fiber.joinable()) {
        fw_fiber.join();
    }

    rs485_6_snooper.reset();
    rs485_2_snooper.reset();
    usb_snooper.reset();
    hw_comms_snooper.reset();
    hw_fallback_snooper.reset();
    spi1_snooper.reset();
    spi3_snooper.reset();

    std::cout << "[Simulation] All fibers joined. Clean exit." << std::endl;
    return 0;
}