#include "NineDOF.h"
#include <MadgwickAHRS.h>
#include <SimpleKalmanFilter.h>

double Net_Accel_9D;
const double GRAVITY = 9.81;

double vertVelocity;
vector<double> position = {0.0, 0.0, 0.0}; // Initial position (x, y, z)
double lastTime = 0.0;                     // Stores the last timestamp
const double gravity = 9.81;               // Gravity in m/s²
const double threshold = 0.05;

SimpleKalmanFilter kalmanAccelX(2, 2, 0.01);
SimpleKalmanFilter kalmanAccelY(2, 2, 0.01);
SimpleKalmanFilter kalmanAccelZ(2, 2, 0.01);

struct SensorCalibration
{
    float ax_offset = 0, ay_offset = 0, az_offset = 0;
    float gx_offset = 0, gy_offset = 0, gz_offset = 0;
} sensorCal;

struct MagCalibration
{
    float x_offset = 0, y_offset = 0, z_offset = 0;
    float x_min = 1000, y_min = 1000, z_min = 1000;
    float x_max = -1000, y_max = -1000, z_max = -1000;
} magCal;

NineDOF::NineDOF() : I2C_9dof(0) {}

bool NineDOF::begin()
{
    Serial.println("Initializing ICM20948...");

    I2C_9dof.begin(36, 37); // SDA, SCL

    if (!icm.begin_I2C(0x69, &I2C_9dof))
    {
        Serial.println("Failed to find ICM20948 chip!");
        while (1)
            delay(10);
    }

    // Serial.println("ICM20948 Found!");
    // filter.begin(500);

    // Serial.println("Calibrating magnetometer...");
    // icm.calibrateMag();
    // Serial.println("Calibration complete!");

    Serial.print("Accelerometer range set to: ");
    switch (icm.getAccelRange())
    {
    case ICM20948_ACCEL_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case ICM20948_ACCEL_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case ICM20948_ACCEL_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case ICM20948_ACCEL_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    Serial.print("Gyro range set to: ");
    switch (icm.getGyroRange())
    {
    case ICM20948_GYRO_RANGE_250_DPS:
        Serial.println("250 degrees/s");
        break;
    case ICM20948_GYRO_RANGE_500_DPS:
        Serial.println("500 degrees/s");
        break;
    case ICM20948_GYRO_RANGE_1000_DPS:
        Serial.println("1000 degrees/s");
        break;
    case ICM20948_GYRO_RANGE_2000_DPS:
        Serial.println("2000 degrees/s");
        break;
    }

    return true;
}

void NineDOF::calibrateSensors()
{
    Serial.println("Place the sensor on a stable surface. Calibration in progress...");

    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float mx_sum = 0, my_sum = 0, mz_sum = 0;

    int numSamples = 1000; // Number of samples for averaging

    for (int i = 0; i < numSamples; i++)
    {
        sensors_event_t accel, gyro, mag, temp;
        icm.getEvent(&accel, &gyro, &temp, &mag);

        // Sum accelerometer values
        ax_sum += accel.acceleration.x;
        ay_sum += accel.acceleration.y;
        az_sum += accel.acceleration.z - 9.81; // Gravity adjustment (assuming Z-up)

        // Sum gyroscope values
        gx_sum += gyro.gyro.x;
        gy_sum += gyro.gyro.y;
        gz_sum += gyro.gyro.z;

        delay(5); // Small delay for stable readings
    }

    // Compute average offsets
    sensorCal.ax_offset = ax_sum / numSamples;
    sensorCal.ay_offset = ay_sum / numSamples;
    sensorCal.az_offset = az_sum / numSamples;

    sensorCal.gx_offset = gx_sum / numSamples;
    sensorCal.gy_offset = gy_sum / numSamples;
    sensorCal.gz_offset = gz_sum / numSamples;

    Serial.println("Calibration Complete!");
    Serial.print("Accelerometer Offsets: X=");
    Serial.print(sensorCal.ax_offset);
    Serial.print(" Y=");
    Serial.print(sensorCal.ay_offset);
    Serial.print(" Z=");
    Serial.println(sensorCal.az_offset);

    Serial.print("Gyroscope Offsets: X=");
    Serial.print(sensorCal.gx_offset);
    Serial.print(" Y=");
    Serial.print(sensorCal.gy_offset);
    Serial.print(" Z=");
    Serial.println(sensorCal.gz_offset);
}

void NineDOF::calibrateMag()
{
    Serial.println("Rotate the sensor in all directions for calibration...");
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) // 10 seconds calibration
    {
        sensors_event_t accel, gyro, mag, temp;
        icm.getEvent(&accel, &gyro, &temp, &mag);

        // Update min/max values
        magCal.x_min = min(magCal.x_min, mag.magnetic.x);
        magCal.y_min = min(magCal.y_min, mag.magnetic.y);
        magCal.z_min = min(magCal.z_min, mag.magnetic.z);

        magCal.x_max = max(magCal.x_max, mag.magnetic.x);
        magCal.y_max = max(magCal.y_max, mag.magnetic.y);
        magCal.z_max = max(magCal.z_max, mag.magnetic.z);
    }

    // Compute offsets
    magCal.x_offset = (magCal.x_max + magCal.x_min) / 2.0;
    magCal.y_offset = (magCal.y_max + magCal.y_min) / 2.0;
    magCal.z_offset = (magCal.z_max + magCal.z_min) / 2.0;

    Serial.println("Magnetometer calibration complete!");
    Serial.print("Offsets - X: ");
    Serial.print(magCal.x_offset);
    Serial.print(" Y: ");
    Serial.print(magCal.y_offset);
    Serial.print(" Z: ");
    Serial.println(magCal.z_offset);
}

void NineDOF::readSensorData()
{
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    // filter.update(
    //     gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
    //     accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
    //     mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    Serial.print("\tAccel X: ");
    Serial.print(accel.acceleration.x - sensorCal.ax_offset);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y - sensorCal.ay_offset);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z - sensorCal.az_offset);
    Serial.println(" m/s^2 ");

    Net_Accel_9D = sqrt((pow(accel.acceleration.x - sensorCal.ax_offset, 2) + pow(accel.acceleration.y - sensorCal.ay_offset, 2) + pow(accel.acceleration.z - sensorCal.az_offset, 2)));

    Serial.print("\t\tMag X: ");
    Serial.print(mag.magnetic.x - magCal.x_offset);
    Serial.print(" \tY: ");
    Serial.print(mag.magnetic.y - magCal.y_offset);
    Serial.print(" \tZ: ");
    Serial.print(mag.magnetic.z - magCal.z_offset);
    Serial.println(" uT");

    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x - sensorCal.gx_offset);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y - sensorCal.gy_offset);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z - sensorCal.gz_offset);
    Serial.println(" radians/s ");

    Serial.println();
}

// need to fix, confused about function
// bool NineDOF::check_IGNITABLE()
// {
//     Serial.print("Net Acceleration: " + String(Net_Accel_9D) + ", ");
//     if (Net_Accel_9D > 68.6)
//     {
//         return true;
//     }
//     else
//     {
//         switch (int(Net_Accel_9D))
//         {
//         default:
//             return true;
//             break;
//         }
//     }
//     return true;
// }

vector<double> NineDOF::getAcceleration()
{
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    Net_Accel_9D = sqrt((pow(accel.acceleration.x - sensorCal.ax_offset, 2) + pow(accel.acceleration.y - sensorCal.ay_offset, 2) + pow(accel.acceleration.z - sensorCal.az_offset, 2)));
    return {accel.acceleration.x - sensorCal.ax_offset, accel.acceleration.y - sensorCal.ay_offset, accel.acceleration.z - sensorCal.az_offset};
}

vector<double> NineDOF::getGyro()
{
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    return {gyro.gyro.x - sensorCal.gx_offset, gyro.gyro.y - sensorCal.gy_offset, gyro.gyro.z - sensorCal.gz_offset};
}

vector<double> NineDOF::getMagno()
{
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    return {mag.magnetic.x - magCal.x_offset, mag.magnetic.y - magCal.y_offset, mag.magnetic.z - magCal.z_offset};
}

double NineDOF::getNetAccel()
{
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    Net_Accel_9D = sqrt((pow(accel.acceleration.x - sensorCal.ax_offset, 2) + pow(accel.acceleration.y - sensorCal.ay_offset, 2) + pow(accel.acceleration.z - sensorCal.az_offset, 2)));
    return Net_Accel_9D;
}

// vector<double> NineDOF::getOrientation()
// {
//     return {filter.getRoll(), filter.getPitch(), filter.getYaw()};
// }

// void NineDOF::updateFilter()
// {
//     sensors_event_t accel, gyro, mag, temp;
//     icm.getEvent(&accel, &gyro, &temp, &mag);
//     // filter.update(
//     //     gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
//     //     accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
//     //     mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
// }

// vector<double> NineDOF::transformToWorldFrame()
// {
//     vector<double> accel = getAcceleration();
//     double roll = filter.getRoll();
//     double pitch = filter.getPitch();
//     double yaw = filter.getYaw();

//     // Convert degrees to radians
//     roll *= M_PI / 180.0;
//     pitch *= M_PI / 180.0;
//     yaw *= M_PI / 180.0;

//     // Compute rotation matrix components
//     double cR = cos(roll), sR = sin(roll);
//     double cP = cos(pitch), sP = sin(pitch);
//     double cY = cos(yaw), sY = sin(yaw);

//     double R[3][3] = {
//         {cY * cP, cY * sP * sR - sY * cR, cY * sP * cR + sY * sR},
//         {sY * cP, sY * sP * sR + cY * cR, sY * sP * cR - cY * sR},
//         {-sP, cP * sR, cP * cR}};

//     // Transform acceleration vector
//     double ax_w = R[0][0] * accel[0] + R[0][1] * accel[1] + R[0][2] * accel[2];
//     double ay_w = R[1][0] * accel[0] + R[1][1] * accel[1] + R[1][2] * accel[2];
//     double az_w = R[2][0] * accel[0] + R[2][1] * accel[1] + R[2][2] * accel[2];

//     return {ax_w, ay_w, az_w};
// }

// // this one does not work
// double NineDOF::updateVerticalMotion(double dt)
// {
//     vector<double> accel_w = transformToWorldFrame();
//     // double dt = currentTime - lastTime;
//     // lastTime = currentTime;

//     if (dt <= 0)
//         return vertVelocity;

//     double az_filtered = accel_w[2] - gravity;

//     if (abs(az_filtered) < threshold)
//         az_filtered = 0.0;

//     vertVelocity += az_filtered * dt;

//     return vertVelocity;
// }
