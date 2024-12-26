/* MPU9250 IMU + Magnetometer (9-axis) 

 Simple code for initialization and parsing of raw IMU data from accelerometer,
 gyroscope, and magnetometer. Send raw data over serial to run filtering offboard,
 which is not practical in reality, but facilitates lower sample rate testing and 
 comparison of multiple attitude filtering strategies.

 Code borrows heavily from Kris Winer's MPU-9250 arduino sketches and filter
 implementation: https://github.com/kriswiner/MPU9250

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "MPU9250.h"

#define SerialDebug false  // Set to true to get Serial output for debugging
#define OUTPUT_INTERVAL_MS \
    4  // Defines interval (rate) at which to write to serial (ms)
const uint8_t HEADER[] = {0xAA, 0xFF};  // Define a 2-byte header

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;   // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS \
    MPU9250_ADDRESS_AD0  // Use either this line or the next to select which I2C
                         // address your device is using
// #define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

// For writing data as bytes to serial
struct ImuState {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
};

ImuState imu_state;

MPU9250 imu(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setup() {
    Wire.begin();
    // TWBR = 12;  // 400 kbit/sec I2C speed
    Serial.begin(115200);

    while (!Serial) {
    };

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Read the WHO_AM_I register, this is a good test of communication
    byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print(F("MPU9250 I AM 0x"));
    Serial.print(c, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x71, HEX);

    if (c == 0x71)  // WHO_AM_I should always be 0x71
    {
        Serial.println(F("MPU9250 is online..."));

        // Start by performing self test and reporting values
        imu.MPU9250SelfTest(imu.selfTest);
        Serial.print(F("x-axis self test: acceleration trim within : "));
        Serial.print(imu.selfTest[0], 1);
        Serial.println("% of factory value");
        Serial.print(F("y-axis self test: acceleration trim within : "));
        Serial.print(imu.selfTest[1], 1);
        Serial.println("% of factory value");
        Serial.print(F("z-axis self test: acceleration trim within : "));
        Serial.print(imu.selfTest[2], 1);
        Serial.println("% of factory value");
        Serial.print(F("x-axis self test: gyration trim within : "));
        Serial.print(imu.selfTest[3], 1);
        Serial.println("% of factory value");
        Serial.print(F("y-axis self test: gyration trim within : "));
        Serial.print(imu.selfTest[4], 1);
        Serial.println("% of factory value");
        Serial.print(F("z-axis self test: gyration trim within : "));
        Serial.print(imu.selfTest[5], 1);
        Serial.println("% of factory value");

        // Calibrate gyro and accelerometers, load biases in bias registers
        imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

        imu.initMPU9250();
        // Initialize device for active mode read of acclerometer, gyroscope,
        // and temperature
        Serial.println("MPU9250 initialized for active data mode....");

        // Read the WHO_AM_I register of the magnetometer, this is a good test
        // of communication
        byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
        Serial.print("AK8963 ");
        Serial.print("I AM 0x");
        Serial.print(d, HEX);
        Serial.print(" I should be 0x");
        Serial.println(0x48, HEX);

        if (d != 0x48) {
            // Communication failed, stop here
            Serial.println(F("Communication failed, abort!"));
            Serial.flush();
            abort();
        }

        // Get magnetometer calibration from AK8963 ROM
        imu.initAK8963(imu.factoryMagCalibration);
        // Initialize device for active mode read of magnetometer
        Serial.println("AK8963 initialized for active data mode....");

        if (SerialDebug) {
            //  Serial.println("Calibration values: ");
            Serial.print("X-Axis factory sensitivity adjustment value ");
            Serial.println(imu.factoryMagCalibration[0], 2);
            Serial.print("Y-Axis factory sensitivity adjustment value ");
            Serial.println(imu.factoryMagCalibration[1], 2);
            Serial.print("Z-Axis factory sensitivity adjustment value ");
            Serial.println(imu.factoryMagCalibration[2], 2);
        }

        // Get sensor resolutions, only need to do this once
        imu.getAres();
        imu.getGres();
        imu.getMres();

        // The following block is output from a recent calibration, using this as hardcoded calibration for now. 
        // Uncomment the calibration call below to run that instead.
        //
        // AK8963 mag biases (mG)
        // -320.81
        // -245.54
        // -6.91
        // AK8963 mag scale (mG)
        // 1.27
        // 0.73
        // 1.19
        imu.magBias[0] = -320.81;
        imu.magBias[1] = -245.54;
        imu.magBias[2] = -6.91;
        imu.magScale[0] = 1.27;
        imu.magScale[1] = 0.73;
        imu.magScale[2] = 1.19;

        // // The next call delays for 4 seconds, and then records about 15 seconds
        // // of data to calculate bias and scale.
        // imu.magCalMPU9250(imu.magBias, imu.magScale);
        // Serial.println("AK8963 mag biases (mG)");
        // Serial.println(imu.magBias[0]);
        // Serial.println(imu.magBias[1]);
        // Serial.println(imu.magBias[2]);

        // Serial.println("AK8963 mag scale (mG)");
        // Serial.println(imu.magScale[0]);
        // Serial.println(imu.magScale[1]);
        // Serial.println(imu.magScale[2]);
        // delay(2000); // Add delay to see results before serial spew of data

        if (SerialDebug) {
            Serial.println("Magnetometer:");
            Serial.print("X-Axis sensitivity adjustment value ");
            Serial.println(imu.factoryMagCalibration[0], 2);
            Serial.print("Y-Axis sensitivity adjustment value ");
            Serial.println(imu.factoryMagCalibration[1], 2);
            Serial.print("Z-Axis sensitivity adjustment value ");
            Serial.println(imu.factoryMagCalibration[2], 2);
        }

    }  // if (c == 0x71)
    else {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);

        // Communication failed, stop here
        Serial.println(F("Communication failed, abort!"));
        Serial.flush();
        abort();
    }
}

void loop() {
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        imu.readAccelData(imu.accelCount);  // Read the x/y/z adc values

        // Read acceleration in milli-Gs
        imu.ax = (float)imu.accelCount[0] * imu.aRes;
        imu.ay = (float)imu.accelCount[1] * imu.aRes;
        imu.az = (float)imu.accelCount[2] * imu.aRes;

        imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values

        // Read gyro data in deg/s
        imu.gx = (float)imu.gyroCount[0] * imu.gRes;
        imu.gy = (float)imu.gyroCount[1] * imu.gRes;
        imu.gz = (float)imu.gyroCount[2] * imu.gRes;

        imu.readMagData(imu.magCount);  // Read the x/y/z adc values

        // Read magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental
        // corrections
        imu.mx =
            (float)imu.magCount[0] * imu.mRes * imu.factoryMagCalibration[0] -
            imu.magBias[0];
        imu.my =
            (float)imu.magCount[1] * imu.mRes * imu.factoryMagCalibration[1] -
            imu.magBias[1];
        imu.mz =
            (float)imu.magCount[2] * imu.mRes * imu.factoryMagCalibration[2] -
            imu.magBias[2];
    }  // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    imu.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientation mismatch in feeding the output to the quaternion filter. For
    // the MPU-9250, we have chosen a magnetic rotation that keeps the sensor
    // forward along the x-axis just like in the LSM9DS0 sensor. This rotation
    // can be modified to allow any convenient orientation convention. This is
    // ok by aircraft orientation standards! Pass gyro rate as rad/s
    // MadgwickQuaternionUpdate(-imu.ay, -imu.ax, imu.az, imu.gy * DEG_TO_RAD,
    //                          imu.gx * DEG_TO_RAD, -imu.gz * DEG_TO_RAD,
    //                          imu.my, imu.mx, imu.mz, imu.deltat);

    imu.delt_t = millis() - imu.count;

    // update serial output once per interval independent of read rate
    if (imu.delt_t > OUTPUT_INTERVAL_MS) {
        // Accelerometer uses RFU coordinates, so we convert to NED
        imu_state.ax = (int)1000 * -imu.ay;
        imu_state.ay = (int)1000 * -imu.ax;
        imu_state.az = (int)1000 * imu.az;

        // Gyroscope uses RFU coordinates, so we need to convert to NED
        imu_state.gx = imu.gy;
        imu_state.gy = imu.gx;
        imu_state.gz = -imu.gz;

        // Magnetometer uses NED coordinates, so no change needed
        imu_state.mx = imu.mx;
        imu_state.my = imu.my;
        imu_state.mz = imu.mz;

        // Write the output to serial; first header, then struct as raw bytes
        Serial.write(HEADER, sizeof(HEADER));
        Serial.write((uint8_t*)&imu_state, sizeof(ImuState));

        if (imu.delt_t > 500 && (SerialDebug)) {
            Serial.print("[ax, ay, az] (mg) = [");
            Serial.print(imu_state.ax, 2);
            Serial.print(", ");
            Serial.print(imu_state.ay, 2);
            Serial.print(", ");
            Serial.print(imu_state.az, 2);
            Serial.println("]");

            Serial.print("[gx, gy, gz] (deg/s) = [");
            Serial.print(imu_state.gy, 2);
            Serial.print(", ");
            Serial.print(imu_state.gx, 2);
            Serial.print(", ");
            Serial.print(imu_state.gz, 2);
            Serial.println("]");

            Serial.print("[mx, my, mz] (mG) = [");
            Serial.print(imu_state.mx);
            Serial.print(", = ");
            Serial.print(imu_state.my);
            Serial.print(", = ");
            Serial.print(imu_state.mz);
            Serial.println("]");

            // Monitor loop rate
            Serial.print("rate = ");
            Serial.print((float)imu.sumCount / imu.sum, 2);
            Serial.println(" Hz");
        }  // if (imu.delt_t > 500 && (SerialDebug))

        // Timer book-keeping
        imu.count = millis();
        imu.sumCount = 0;
        imu.sum = 0;
    }  // if (imu.delt_t > OUTPUT_INTERVAL_MS)
}
