#include "LaunchState.h"
#include <Arduino.h>
// #include "SixDOF.h"
#include "PHT.h"
// #include "MPU9250.h"
#include "GPS.h"
#include <map>
#include <vector>
using namespace std;
#define RATE_THRESHOLD 0.05
#define ERROR_RANGE 25
#define HEIGHT_OFFSET 5 // correct this
// values below should be fixed
#define EXPECTED_NETACCEL 100
#define IGNITE_THRESHOLD 15

const double FAIL_TIME = 30000;

const double EXPECTED_ALTITUDE = 1000;
const double EXPECTED_ACCELERATION = 9.8;
LaunchState current_state = LaunchState::PreIgnition;
const int READINGS_LENGTH = 15;
double rate_of_change_1;
double rate_of_change_2;
double AltArray[READINGS_LENGTH];
double AltArray2[READINGS_LENGTH];
double VelArray[READINGS_LENGTH];
double VelArray2[READINGS_LENGTH];
int count = 0;
double previousMedian = EXPECTED_ALTITUDE;

double groundLevelAltitude;
std::map<string, bool> statesMap;

double returnAverage(double arr[], int number)
{
    double sum = 0;
    for (int count = 0; count < number - 1; count++)
    {
        sum += arr[count];
    }
    return sum / number;
}

double returnMedian(double arr[], int number)
{
    double temp[number];
    memcpy(temp, arr, sizeof(temp));
    std::sort(temp, temp + number);

    if (number % 2 == 0)
    {
        return (temp[number / 2 - 1] + temp[number / 2]) / 2.0;
    }
    else
    {
        return temp[number / 2];
    }
}

double calculateRateOfChange(double AltArray[], int READINGS_LENGTH)
{
    double currentMedian = returnMedian(AltArray, READINGS_LENGTH);

    double rate_of_change = currentMedian - previousMedian;

    previousMedian = currentMedian;

    return rate_of_change;
}

// bool check_IGNITABLE(SixDOF &_6DOF, PHT &alt)
// {
//     double netAccel = _6DOF.getNetAccel();
//     if (fabs(netAccel - EXPECTED_NETACCEL) < IGNITE_THRESHOLD)
//     {
//     }
//     // check mpu against 7 Gs
//     // check altitude rate of change against 10 m/s
//     return false;
// }
// in the main loop, we will get sensor data from 6dof, altimeter, and mpu.
// then we will run HalyaStateMachine, which can determine correct state of the flight and run prioritized code for that state
void HalyaStateMachine(PHT &alt1, GPS &gps1, PHT &alt2, GPS &gps2)
{
    static int retryCount = 0;
    const int MAX_RETRIES = 5;

    switch (current_state)
    {

    // read from serial and enable movement across launch states
    case LaunchState::PreIgnition:
    {
        // while (!_6DOF1.check_IGNITABLE())
        // {
        //     delay(4000); // Long delay to conserve power
        // }

        // bool is_6DOF_working = _6DOF1.checkReadings();
        bool is_altimeter_working = alt1.getAltitude() != 0;
        // bool is_mpu_working = mpu1.update() && mpu1.getAcc(millis()) != 0 & mpu1.getGyro(millis()) != 0;
        bool is_gps_working = gps1.readingCheck();

        if (is_altimeter_working & is_gps_working)
        {
            groundLevelAltitude = alt1.getAltitude();
            // statesMap["_6DOF"] = true;
            statesMap["altimeter"] = true;
            // statesMap["mpu"] = true;
            statesMap["gps"] = true;
        }
        else
        {
            if (!is_altimeter_working)
                Serial.println("Warning: Altimeter failure.");
            // if (!is_mpu_working)
            //     Serial.println("Warning: MPU failure.");

            delay(20);
            retryCount++;
            if (retryCount == MAX_RETRIES)
            {
                Serial.print("Continuing with limited functionality.");
                statesMap["altimeter"] = is_altimeter_working;
                // statesMap["mpu"] = is_mpu_working;
                statesMap["gps"] = is_gps_working;
            }
        }

        current_state = LaunchState::Ignition_to_Apogee;
        Serial.println("halya ignition!");
        break;
    }

    case LaunchState::Ignition_to_Apogee:
    {
        static int count = 0;

        // Read sensor values
        double PHT1_alt = alt1.getAltitude();
        double PHT2_alt = alt2.getAltitude() + HEIGHT_OFFSET;
        double GPS1_alt = gps1.getAltitude();
        double GPS2_alt = gps2.getAltitude() + HEIGHT_OFFSET;

        // Error detection
        bool PHT1_error = (PHT1_alt == 0);
        bool PHT2_error = (PHT2_alt == 0);
        bool GPS1_error = gps1.readingCheck();
        bool GPS2_error = gps2.readingCheck();

        // Valid altitudes (non-error)
        std::vector<double> validReadings;
        if (!PHT1_error)
            validReadings.push_back(PHT1_alt);
        if (!PHT2_error)
            validReadings.push_back(PHT2_alt);
        if (!GPS1_error)
            validReadings.push_back(GPS1_alt);
        if (!GPS2_error)
            validReadings.push_back(GPS2_alt);

        double final_altitude = 0;

        if (validReadings.size() >= 3)
        {
            // Find the median of the valid readings
            std::sort(validReadings.begin(), validReadings.end());
            double median_alt = (validReadings.size() % 2 == 0) ? (validReadings[validReadings.size() / 2 - 1] + validReadings[validReadings.size() / 2]) / 2.0
                                                                : validReadings[validReadings.size() / 2];

            // Check if three sensors agree within ERROR_RANGE
            int agreement_count = 0;
            for (double alt : validReadings)
            {
                if (fabs(alt - median_alt) < ERROR_RANGE)
                {
                    agreement_count++;
                }
            }

            if (agreement_count >= 3)
            {
                final_altitude = median_alt; // Ignore the outlier
            }
            else
            {
                // If two agree and two don't, favor the top bay sensors (PHT1, GPS1)
                if ((fabs(PHT1_alt - GPS1_alt) < ERROR_RANGE) && (!PHT1_error && !GPS1_error))
                {
                    final_altitude = (PHT1_alt + GPS1_alt) / 2.0;
                }
                else if ((fabs(PHT2_alt - GPS2_alt) < ERROR_RANGE) && (!PHT2_error && !GPS2_error))
                {
                    final_altitude = (PHT2_alt + GPS2_alt) / 2.0;
                }
                else
                {
                    // Otherwise, prioritize PHT sensors
                    if (!PHT1_error && !PHT2_error)
                    {
                        final_altitude = (PHT1_alt + PHT2_alt) / 2.0;
                    }
                    else if (!PHT1_error)
                    {
                        final_altitude = PHT1_alt;
                    }
                    else if (!PHT2_error)
                    {
                        final_altitude = PHT2_alt;
                    }
                    else
                    {
                        // Last fallback to GPS
                        if (!GPS1_error && !GPS2_error)
                        {
                            final_altitude = (GPS1_alt + GPS2_alt) / 2.0;
                        }
                        else if (!GPS1_error)
                        {
                            final_altitude = GPS1_alt;
                        }
                        else if (!GPS2_error)
                        {
                            final_altitude = GPS2_alt;
                        }
                    }
                }
            }
        }
        else
        {
            // Not enough valid readings, fallback logic
            if (!PHT1_error && !PHT2_error)
            {
                final_altitude = (PHT1_alt + PHT2_alt) / 2.0;
            }
            else if (!PHT1_error)
            {
                final_altitude = PHT1_alt;
            }
            else if (!PHT2_error)
            {
                final_altitude = PHT2_alt;
            }
            else if (!GPS1_error && !GPS2_error)
            {
                final_altitude = (GPS1_alt + GPS2_alt) / 2.0;
            }
            else if (!GPS1_error)
            {
                final_altitude = GPS1_alt;
            }
            else if (!GPS2_error)
            {
                final_altitude = GPS2_alt;
            }
        }

        // Store the altitude values for rate of change calculation
        AltArray[count % READINGS_LENGTH] = final_altitude;
        count++;

        // Step 5: Check for apogee based on calculated rate of change
        if (count >= READINGS_LENGTH)
        {
            double rate_of_change_1 = calculateRateOfChange(AltArray, READINGS_LENGTH);
            // double rate_of_change_2 = calculateRateOfChange(AltArray2, READINGS_LENGTH);

            if ((fabs(rate_of_change_1) < RATE_THRESHOLD && rate_of_change_1 < 0))
            {
                Serial.println("Halya has reached apogee!");
                current_state = LaunchState::Thousand_ft;
            }
            count = 0;
        }

        delay(100); // Delay for sensor updates
        break;
    }

    case LaunchState::Thousand_ft:
    {
        bool PHT1_error = (alt1.getAltitude() == 0);
        bool PHT2_error = (alt2.getAltitude() == 0);
        bool GPS1_error = gps1.readingCheck();
        bool GPS2_error = gps2.readingCheck();
        static unsigned long startTime = millis(); // Track the start time of this state

        double altitudeReading = 0;
        bool usePHT1 = !PHT1_error;
        bool usePHT2 = !PHT2_error;

        // Step 1: Check if either PHT is working and use the lowest working altitude
        if (!PHT1_error && !PHT2_error)
        {
            altitudeReading = min(alt1.getAltitude(), alt2.getAltitude());
        }
        else if (usePHT1)
        {
            altitudeReading = alt1.getAltitude();
        }
        else if (usePHT2)
        {
            altitudeReading = alt2.getAltitude();
        }
        // Step 2: If both PHTs fail, use GPS as fallback
        else if (!GPS1_error && !GPS2_error)
        {
            altitudeReading = min(gps1.getAltitude(), gps2.getAltitude());
        }
        else if (!GPS1_error)
        {
            altitudeReading = gps1.getAltitude();
        }
        else if (!GPS2_error)
        {
            altitudeReading = gps2.getAltitude();
        }
        // Step 3: If no sensor data is valid, fall back to timed deployment after 30 seconds
        else if (millis() - startTime >= FAIL_TIME)
        {
            Serial.println("No altitude data - deploying parachute based on timeout.");
            // deployMainParachute();
            current_state = LaunchState::Descent;
            break;
        }

        // Step 4: Deploy parachute based on altitude thresholds
        if (altitudeReading <= 1750)
        {
            Serial.println("Reached 1750 meters - preparing for parachute deployment.");
            // deploy parachute
            current_state = LaunchState::Descent;
        }
        else if (altitudeReading <= 1500)
        {
            Serial.println("Reached 1500 meters - deploying parachute.");
            // deploy parachute
            current_state = LaunchState::Descent;
        }
        else if (altitudeReading <= 1250)
        {
            Serial.println("Reached 1250 meters - final check for parachute deployment.");
            // deploy parachute
            current_state = LaunchState::Descent;
        }

        break;
    }

    case LaunchState::Descent:
    {
        current_state = LaunchState::Touchdown;
        break;
        // send GPS data
    }

    case LaunchState::Touchdown:
    {
        // Print GPS readings if required
        // send GPS data
        break;
    }
    }
}
