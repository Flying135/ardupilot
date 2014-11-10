/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6050_H__
#define __AP_INERTIAL_SENSOR_MPU6050_H__

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include <stdint.h>

#include <AP_Math.h>
#include <AP_Progmem.h>
#include "AP_InertialSensor.h"

// enable debug to see a register dump on startup
#define MPU6050_DEBUG 0

// on fast CPUs we sample at 1kHz and use a software filter
#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define MPU6050_FAST_SAMPLING 1
#else
#define MPU6050_FAST_SAMPLING 0
#endif

#if MPU6050_FAST_SAMPLING
#include <Filter.h>
#include <LowPassFilter2p.h>
#endif

class AP_InertialSensor_MPU6050 : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_MPU6050(AP_InertialSensor &imu);

    /* update accel and gyro state */
    bool update();

    bool gyro_sample_available(void) { return _sum_count >= _sample_count; }
    bool accel_sample_available(void) { return _sum_count >= _sample_count; }

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:
#if MPU6050_DEBUG
    void _dump_registers(void);
#endif

    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

#ifdef MPU6050_DRDY_PIN
    AP_HAL::DigitalSource *_drdy_pin;
#endif

    bool                 _init_sensor(void);
    bool                 _sample_available();
    void                 _read_data_transaction();
    bool                 _data_ready();
    void                 _poll_data(void);
    uint8_t              _register_read( uint8_t reg );
    void                 _register_write( uint8_t reg, uint8_t val );
    void                 _register_write_check(uint8_t reg, uint8_t val);
    bool                 _hardware_init(void);

    AP_HAL::I2CDriver *_i2c;
    AP_HAL::Semaphore *_i2c_sem;

    static const float          _gyro_scale;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;

    void _set_filter_register(uint8_t filter_hz);

    // count of bus errors
    uint16_t _error_count;

    // how many hardware samples before we report a sample to the caller
    uint8_t _sample_count;

#if MPU6050_FAST_SAMPLING
    Vector3f _accel_filtered;
    Vector3f _gyro_filtered;

    // Low Pass filters for gyro and accel
    LowPassFilter2p _accel_filter_x;
    LowPassFilter2p _accel_filter_y;
    LowPassFilter2p _accel_filter_z;
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;
#else
    // accumulation in timer - must be read with timer disabled
    // the sum of the values since last read
    Vector3l _accel_sum;
    Vector3l _gyro_sum;
#endif
    volatile uint16_t _sum_count;
};

#endif
#endif // __AP_INERTIAL_SENSOR_MPU6050_H__
