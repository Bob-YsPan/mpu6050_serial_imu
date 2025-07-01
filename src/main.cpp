#include <Arduino.h>
#include <MPU6050.h>

// MPU6050 device
MPU6050 mpu;
// 暫存測量到的資料
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;
// 沒有磁力資料
int16_t hx = 0;
int16_t hy = 0;
int16_t hz = 0;
// 切換燈號用變數
bool sign = true;
unsigned long lastSwitchTime = 0;
unsigned long lastSendTime = 0;

// Usage:
// value: variable of the address of the variable need to be send
// temp_chksum: variable of the address of the variable that summing the checksum
void sendAndChecksum(int16_t* value, unsigned int* temp_chksum)
{
    // Dereference the value pointer to get the actual int16_t number.
    int16_t actualValue = *value;

    // Handle negative values by mapping them to a positive range
    // (e.g., for specific protocol requirements where negative numbers
    // are represented differently).
    if (actualValue < 0) {
        actualValue = 32768 - actualValue; // This converts negative to positive range
    }

    // Extract the high byte (most significant 8 bits)
    uint8_t highByte = (actualValue >> 8) & 0xFF;
    Serial.write(highByte); // Send the high byte via Serial
    // Add the high byte to the checksum, dereferencing the pointer.
    *temp_chksum += highByte;

    // Extract the low byte (least significant 8 bits)
    uint8_t lowByte = actualValue & 0xFF;
    Serial.write(lowByte); // Send the low byte via Serial
    // Add the low byte to the checksum, dereferencing the pointer.
    *temp_chksum += lowByte;
}

// The reportMotion function as defined above
void reportMotion(int16_t ax, int16_t ay, int16_t az,
                  int16_t gx, int16_t gy, int16_t gz,
                  int16_t hx, int16_t hy, int16_t hz)
{
    unsigned int temp = 0xaF + 9;

    Serial.write(0xa5);
    Serial.write(0x5a);
    Serial.write(22);   // Packet length of length byte itself + all data + chksum + footer (No header)
    Serial.write(0xA2);

    sendAndChecksum(&ax, &temp);
    sendAndChecksum(&ay, &temp);
    sendAndChecksum(&az, &temp);
    sendAndChecksum(&gx, &temp);
    sendAndChecksum(&gy, &temp);
    sendAndChecksum(&gz, &temp);
    sendAndChecksum(&hx, &temp);
    sendAndChecksum(&hy, &temp);
    sendAndChecksum(&hz, &temp);

    Serial.write(temp % 256);
    Serial.write(0xaa);
}

// --- Arduino Setup and Loop ---

void setup()
{
    // Initialize serial communication at 115200 bits per second:
    // Make sure this baud rate matches the host computer's expectation.
    Serial.begin(115200);
    Wire.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);
    // Init sensor
    mpu.initialize();
    // Hint user sensor init
    digitalWrite(LED_BUILTIN, 1);
    if(mpu.testConnection() == false){
        // 燈快閃代表初始化失敗
        while(true){
            digitalWrite(LED_BUILTIN, 0);
            delay(100);
            digitalWrite(LED_BUILTIN, 1);
            delay(100);
            digitalWrite(LED_BUILTIN, 0);
            delay(100);
            digitalWrite(LED_BUILTIN, 1);
            delay(100);
            digitalWrite(LED_BUILTIN, 0);
            delay(1000);
        }
    }
    // set scale range
    // 2g Range = 16384 LSB/g
    // 2000 deg/s = 16.4 LSB/deg/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    // Set offset
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    // Calibration
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    // All init
    digitalWrite(LED_BUILTIN, 0);
}

void loop()
{
    unsigned long currentTime = millis();
    // 每 1 秒切換燈的亮滅(Heartbeat)
    if (currentTime - lastSwitchTime > 1000) {
        sign = !sign;
        digitalWrite(LED_BUILTIN, sign);
        lastSwitchTime = currentTime;
    }
    // Grab data each 10ms, limit rate to about 100Hz
    if (currentTime - lastSendTime > 10) {
        // 取得加速度資料
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // 送出加速度資料
        reportMotion(ax, ay, az, gx, gy, gz, hx, hy, hz);
        // Reset timer
        lastSendTime = currentTime;
    }
}