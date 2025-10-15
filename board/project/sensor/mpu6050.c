#include "mpu6050.h"

#include <math.h>
#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define REGISTER_PWR_MGMT_1 0x6B
#define REGISTER_SMPLRT_DIV 0x19
#define REGISTER_CONFIG 0x1A
#define REGISTER_GYRO_CONFIG 0x1B
#define REGISTER_ACCEL_CONFIG 0x1C
#define REGISTER_INT_ENABLE 0x38
#define REGISTER_ACCEL_XOUT_H 0x3B
#define REGISTER_ACCEL_XOUT_L 0x3C
#define REGISTER_ACCEL_YOUT_H 0x3D
#define REGISTER_ACCEL_YOUT_L 0x3E
#define REGISTER_ACCEL_ZOUT_H 0x3F
#define REGISTER_ACCEL_ZOUT_L 0x40
#define REGISTER_TEMP_OUT_H 0x41
#define REGISTER_TEMP_OUT_L 0x42
#define REGISTER_GYRO_XOUT_H 0x43
#define REGISTER_GYRO_XOUT_L 0x44
#define REGISTER_GYRO_YOUT_H 0x45
#define REGISTER_GYRO_YOUT_L 0x46
#define REGISTER_GYRO_ZOUT_H 0x47
#define REGISTER_GYRO_ZOUT_L 0x48
#define REGISTER_WHO_AM_I 0x75

#define SENSOR_READ_INTERVAL_MS 30
#define CALIBRATION_READS 50

typedef struct mpu6050_calibration_t {
    float acc_error_x, acc_error_y;
    float gyro_error_x, gyro_error_y, gyro_error_z;
} mpu6050_calibration_t;

typedef struct mpu6050_acceleration_t {
    float x, y, z;
} mpu6050_acceleration_t;

typedef struct mpu6050_gyro_t {
    float x, y, z;
} mpu6050_gyro_t;

static void read(mpu6050_parameters_t *parameter, mpu6050_calibration_t *calibration, uint accel_divider,
                 uint gyro_divider);
static mpu6050_acceleration_t read_acc(uint8_t address, float accel_divider);
static mpu6050_gyro_t read_gyro(uint8_t address, float gyro_divider);
static void begin(mpu6050_parameters_t *parameter, mpu6050_calibration_t *calibration, float accel_divider,
                  float gyro_divider);
static void calibrate_imu(uint8_t address, mpu6050_calibration_t *calibration, float accel_divider, float gyro_divider);

void mpu6050_task(void *parameters) {
    mpu6050_parameters_t parameter = *(mpu6050_parameters_t *)parameters;
    xTaskNotifyGive(context.receiver_task_handle);
    *parameter.roll = 0;
    *parameter.pitch = 0;
    *parameter.yaw = 0;
    *parameter.acc_x = 0;
    *parameter.acc_y = 0;
    *parameter.acc_z = 0;
    *parameter.acc = 0;

    mpu6050_calibration_t calibration = {0};
    vTaskDelay(500 / portTICK_PERIOD_MS);

    uint accel_divider, gyro_divider;
    switch (parameter.acc_scale) {
        case 0:
            accel_divider = 16384;  // +-2g
            break;
        case 1:
            accel_divider = 8192;  // +-4g
            break;
        case 2:
            accel_divider = 4096;  // +-8g
            break;
        case 3:
            accel_divider = 2048;  // +-16g
            break;
        default:
            accel_divider = 16384;  // Default to +-2g
            break;
    }

    switch (parameter.gyro_scale) {
        case 0:
            gyro_divider = 131;  // 250deg/s
            break;
        case 1:
            gyro_divider = 65.5;  // 500deg/s
            break;
        case 2:
            gyro_divider = 32.8;  // 1000deg/s
            break;
        case 3:
            gyro_divider = 16.4;  // 2000deg/s
            break;
        default:
            gyro_divider = 131;  // Default to 250deg/s
            break;
    }

    begin(&parameter, &calibration, accel_divider, gyro_divider);

    while (1) {
        read(&parameter, &calibration, accel_divider, gyro_divider);
        debug(
            "\nMPU6050 (%u) < Roll: %.2f Pitch: %.2f Yaw: %.2f",
            uxTaskGetStackHighWaterMark(NULL), calibration.acc_error_x, calibration.acc_error_y,
           calibration.gyro_error_x, calibration.gyro_error_y, calibration.gyro_error_z, *parameter.roll,
           *parameter.pitch, *parameter.yaw);
        vTaskDelay(SENSOR_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

static void read(mpu6050_parameters_t *parameter, mpu6050_calibration_t *calibration, uint accel_divider,
                 uint gyro_divider) {
    uint8_t data[6] = {0};
    float acc_angle_x, acc_angle_y;
    static float gyro_angle_x = 0, gyro_angle_y = 0;
    static uint32_t lastTimestamp = 0;

    // Read accelerometer data
    mpu6050_acceleration_t acc = read_acc(parameter->address, accel_divider);
    acc_angle_x = (atan(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2))) * 180 / PI);
    acc_angle_y = (atan(-1 * acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180 / PI);
    acc_angle_x -= calibration->acc_error_x;
    acc_angle_y -= calibration->acc_error_y;
    *parameter->acc_x = acc.x;
    *parameter->acc_y = acc.y;
    *parameter->acc_z = acc.z;
    *parameter->acc = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

    // Read gyroscope data
    mpu6050_gyro_t gyro = read_gyro(parameter->address, gyro_divider);
    gyro.x -= calibration->gyro_error_x;
    gyro.y -= calibration->gyro_error_y;
    gyro.z -= calibration->gyro_error_z;
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the
    // angle in degrees
    uint32_t now = time_us_32();
    float elapsedTime = (now - lastTimestamp) / 1000000.0;
    gyro_angle_x += gyro.x * elapsedTime;
    gyro_angle_y += gyro.y * elapsedTime;

    // Combine accelerometer and gyro angle values
    *parameter->roll =
        parameter->gyro_weighting / 100.0 * gyro_angle_x + (100 - parameter->gyro_weighting) / 100.0 * acc_angle_x;
    *parameter->pitch =
        parameter->gyro_weighting / 100.0 * gyro_angle_y + (100 - parameter->gyro_weighting) / 100.0 * acc_angle_y;
    *parameter->yaw += gyro.z * elapsedTime;

    lastTimestamp = now;

#ifdef SIM_SENSORS
    *parameter->roll = 12.34;
    *parameter->pitch = 23.56;
    *parameter->yaw = 45.67;
#endif
}

static mpu6050_acceleration_t read_acc(uint8_t address, float accel_divider) {
    uint8_t data[6] = {0};
    mpu6050_acceleration_t acc;

    // Read accelerometer data
    data[0] = REGISTER_ACCEL_XOUT_H;
    i2c_write_blocking(i2c0, address, data, 1, true);
    i2c_read_blocking(i2c0, address, data, 6, false);
    acc.x = (int16_t)(data[0] << 8 | data[1]) / accel_divider;
    acc.y = (int16_t)(data[2] << 8 | data[3]) / accel_divider;
    acc.z = (int16_t)(data[4] << 8 | data[5]) / accel_divider;

    // debug("\nAcc: X: %5.2f Y: %5.2f Z: %5.2f", acc.x, acc.y, acc.z);

    return acc;
}

static mpu6050_gyro_t read_gyro(uint8_t address, float gyro_divider) {
    uint8_t data[6] = {0};
    mpu6050_gyro_t gyro;

    // Read gyroscope data
    data[0] = REGISTER_GYRO_XOUT_H;
    i2c_write_blocking(i2c0, address, data, 1, true);
    i2c_read_blocking(i2c0, address, data, 6, false);
    gyro.x = (int16_t)(data[0] << 8 | data[1]) / gyro_divider;
    gyro.y = (int16_t)(data[2] << 8 | data[3]) / gyro_divider;
    gyro.z = (int16_t)(data[4] << 8 | data[5]) / gyro_divider;

    // debug("\nGyro: X: %5.2f Y: %5.2f Z: %5.2f", gyro.x, gyro.y, gyro.z);

    return gyro;
}

static void begin(mpu6050_parameters_t *parameter, mpu6050_calibration_t *calibration, float accel_divider,
                  float gyro_divider) {
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C0_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_GPIO);
    gpio_pull_up(I2C0_SCL_GPIO);

    // Reset
    uint8_t data[2] = {REGISTER_PWR_MGMT_1, 0};
    i2c_write_blocking(i2c0, parameter->address, data, 2, true);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Configure Accelerometer Sensitivity
    data[0] = REGISTER_ACCEL_CONFIG;
    data[1] = parameter->acc_scale << 3;
    i2c_write_blocking(i2c0, parameter->address, data, 2, true);

    // Configure Gyroscope Sensitivity
    data[0] = REGISTER_GYRO_CONFIG;
    data[1] = parameter->gyro_scale << 3;
    i2c_write_blocking(i2c0, parameter->address, data, 2, true);

    // Set sample rate to 1kHz by writing SMPLRT_DIV
    // data[0] = REGISTER_SMPLRT_DIV;
    // data[1] = 0x07;  // Use a 1kHz gyroscope rate, set SMPLRT_DIV to 7 for a 125Hz sample rate
    // i2c_write_blocking(i2c0, parameter->address, data, 2, true);

    // Set Digital Low Pass Filter
    data[0] = REGISTER_CONFIG;
    data[1] = parameter->filter;
    i2c_write_blocking(i2c0, parameter->address, data, 2, true);

    // Calibrate IMU
    calibrate_imu(parameter->address, calibration, accel_divider, gyro_divider);
}

static void calibrate_imu(uint8_t address, mpu6050_calibration_t *calibration, float accel_divider,
                          float gyro_divider) {
    // Read accelerometer values
    for (uint i = 0; i < CALIBRATION_READS; i++) {
        mpu6050_acceleration_t acc = read_acc(address, accel_divider);
        calibration->acc_error_x += ((atan((acc.y) / sqrt(pow((acc.x), 2) + pow((acc.z), 2))) * 180 / PI));
        calibration->acc_error_y += ((atan(-1 * (acc.x) / sqrt(pow((acc.y), 2) + pow((acc.z), 2))) * 180 / PI));
        vTaskDelay(SENSOR_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
    // Divide the sum to get the error value
    calibration->acc_error_x /= CALIBRATION_READS;
    calibration->acc_error_y /= CALIBRATION_READS;

    // Read gyro values
    for (uint i = 0; i < CALIBRATION_READS; i++) {
        mpu6050_gyro_t gyro = read_gyro(address, gyro_divider);
        calibration->gyro_error_x += gyro.x;
        calibration->gyro_error_y += gyro.y;
        calibration->gyro_error_z += gyro.z;
        vTaskDelay(SENSOR_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
    // Divide the sum to get the error value
    calibration->gyro_error_x /= CALIBRATION_READS;
    calibration->gyro_error_y /= CALIBRATION_READS;
    calibration->gyro_error_z /= CALIBRATION_READS;

    debug("\nMPU6050. Calibration. AccX: %.2f AccY: %.2f GyroX: %.2f GyroY: %.2f GyroZ: %.2f", calibration->acc_error_x, calibration->acc_error_y,
          calibration->gyro_error_x, calibration->gyro_error_y, calibration->gyro_error_z);
}