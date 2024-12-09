/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

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
#include "quaternionFilters.h"

#define AHRS true          // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;   // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS \
    MPU9250_ADDRESS_AD0  // Use either this line or the next to select which I2C
                         // address your device is using
// #define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

// Places the angle on the interval (-180, 180] degrees.
float normalize_angle_deg(float angle) {
    // Normalize the angle to the range [-360, 360) first
    angle = fmod(angle, 360.0);
    if (angle <= -180.0) {
        angle += 360.0;
    } else if (angle > 180.0) {
        angle -= 360.0;
    }
    return angle;
}

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

        // The next call delays for 4 seconds, and then records about 15 seconds
        // of data to calculate bias and scale.
        //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
        Serial.println("AK8963 mag biases (mG)");
        Serial.println(imu.magBias[0]);
        Serial.println(imu.magBias[1]);
        Serial.println(imu.magBias[2]);

        Serial.println("AK8963 mag scale (mG)");
        Serial.println(imu.magScale[0]);
        Serial.println(imu.magScale[1]);
        Serial.println(imu.magScale[2]);
        //    delay(2000); // Add delay to see results before serial spew of
        //    data

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

        // Now we'll calculate the accleration value into actual g's
        // This depends on scale being set
        imu.ax = (float)imu.accelCount[0] * imu.aRes;  // - myIMU.accelBias[0];
        imu.ay = (float)imu.accelCount[1] * imu.aRes;  // - myIMU.accelBias[1];
        imu.az = (float)imu.accelCount[2] * imu.aRes;  // - myIMU.accelBias[2];

        imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        imu.gx = (float)imu.gyroCount[0] * imu.gRes;
        imu.gy = (float)imu.gyroCount[1] * imu.gRes;
        imu.gz = (float)imu.gyroCount[2] * imu.gRes;

        imu.readMagData(imu.magCount);  // Read the x/y/z adc values

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental
        // corrections
        // Get actual magnetometer value, this depends on scale being set
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
    MadgwickQuaternionUpdate(-imu.ay, -imu.ax, imu.az, imu.gy * DEG_TO_RAD,
                             imu.gx * DEG_TO_RAD, -imu.gz * DEG_TO_RAD, imu.my,
                             imu.mx, imu.mz, imu.deltat);

    // Serial print and/or display at 0.5 s rate independent of data rates
    imu.delt_t = millis() - imu.count;

    // update serial output once per 500ms independent of read rate
    if (imu.delt_t > 500) {
        if (SerialDebug) {
            // Accelerometer uses RFU coordinates, so we convert to NED
            Serial.print("[ax, ay, az] (mg) = [");
            Serial.print((int)1000 * -imu.ay);
            Serial.print(", ");
            Serial.print((int)1000 * -imu.ax);
            Serial.print(", ");
            Serial.print((int)1000 * imu.az);
            Serial.println("]");

            // Gyroscope uses RFU coordinates, so we need to convert to NED
            Serial.print("[gx, gy, gz] (deg/s) = [");
            Serial.print(imu.gy, 2);
            Serial.print(", ");
            Serial.print(imu.gx, 2);
            Serial.print(", ");
            Serial.print(-imu.gz, 2);
            Serial.println("]");

            Serial.print("[mx, my, mz] (mG) = [");
            Serial.print((int)imu.mx);
            Serial.print(", = ");
            Serial.print((int)imu.my);
            Serial.print(", = ");
            Serial.print((int)imu.mz);
            Serial.println("]");

            // Serial.print("q0 = ");
            // Serial.print(*getQ());
            // Serial.print(" qx = ");
            // Serial.print(*(getQ() + 1));
            // Serial.print(" qy = ");
            // Serial.print(*(getQ() + 2));
            // Serial.print(" qz = ");
            // Serial.println(*(getQ() + 3));
        }

        // Define output variables from updated quaternion---these are
        // Tait-Bryan angles, commonly used in aircraft orientation. In this
        // coordinate system, the positive z-axis is down toward Earth. Yaw
        // is the angle between Sensor x-axis and Earth magnetic North (or
        // true North if corrected for local declination, looking down on
        // the sensor positive yaw is counterclockwise. Pitch is angle
        // between sensor x-axis and Earth ground plane, toward the Earth is
        // positive, up toward the sky is negative. Roll is angle between
        // sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation
        // matrix constructed from quaternions. Tait-Bryan angles as well as
        // Euler angles are non-commutative; that is, the get the correct
        // orientation the rotations must be applied in the correct order
        // which for this configuration is yaw, pitch, and then roll. For
        // more see
        // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // which has additional links.
        imu.yaw = atan2(
            2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)),
            *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) -
                *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
        imu.pitch = -asin(
            2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
        imu.roll = atan2(
            2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)),
            *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) -
                *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
        imu.yaw *= RAD_TO_DEG;
        imu.pitch *= RAD_TO_DEG;
        imu.roll *= RAD_TO_DEG;

        // Apply declination correction
        // Ref: http://www.ngdc.noaa.gov/geomag-web/#declination
        imu.yaw -= 13.25;  // san francisco: 37° 46' 37"N, 122° 25' 10"W

        // Normalize
        imu.yaw = normalize_angle_deg(imu.yaw);
        imu.pitch = normalize_angle_deg(imu.pitch);
        imu.roll = normalize_angle_deg(imu.roll);

        if (SerialDebug) {
            Serial.print("rate = ");
            Serial.print((float)imu.sumCount / imu.sum, 2);
            Serial.println(" Hz");
        }

        // send ypr to serial
        Serial.print("ypr: ");
        Serial.print(imu.yaw, 3);
        Serial.print(", ");
        Serial.print(imu.pitch, 3);
        Serial.print(", ");
        Serial.println(imu.roll, 3);
        // Serial.println("]");

        imu.count = millis();
        imu.sumCount = 0;
        imu.sum = 0;
    }  // if (imu.delt_t > 500)
}
