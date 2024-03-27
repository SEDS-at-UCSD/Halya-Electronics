#include "LaunchState.h"
#include <Arduino.h>
#include "SixDOF.h"
#include "PHT.h"
#include "MPU9250.h"

LaunchState current_state = LaunchState::PreIgnition;
const int READINGS_LENGTH = 10;
double rate_of_change;
double AltArray[READINGS_LENGTH];
int count = 0;

double returnAverage(double arr[], int number) {
    int sum = 0;
    for(count = 0; count < number-1; count++) {
        sum += arr[count];
    }
    return sum/number;
}
//in the main loop, we will get sensor data from 6dof, altimeter, and mpu. 
//then we will run HalyaStateMachine, which can determine correct state of the flight and run prioritized code for that state
void HalyaStateMachine(SixDOF &_6DOF, PHT &alt, MPU9250 &mpu) {
    switch (current_state) {


        case LaunchState::PreIgnition:
            while(!_6DOF.check_IGNITABLE()){
                delay(4000); //long delay to not waste power
            }
            Serial.println("halya ignition!");
            current_state = LaunchState::Ignition;
            break;

//**IGNITION STAGE SEEMS REDUNDANT, combine with preignition state

        case LaunchState::Ignition:                  
            //fill AltArray and check rate of change
            if (count < READINGS_LENGTH) {
                AltArray[count] = alt.getAltitude();
                } 
            count++;
            if(count >= READINGS_LENGTH) {
                for (int i = 0; i < READINGS_LENGTH - 1; i++){
                    AltArray[i] = AltArray[i+1];
                }
                AltArray[READINGS_LENGTH - 1] = alt.getAltitude(); //does altitude update in the loop? I think that it would just set everything in AltArray to the same value
                rate_of_change = returnAverage(AltArray, READINGS_LENGTH);
            }
            Serial.println("halya ignition!");
            current_state = LaunchState::Apogee;
            break;

        case LaunchState::Apogee :
            if(rate_of_change < 0){
                // digitalWrite(igniter1, HIGH);
                delay(1000);
            }
            Serial.println("halya has reached apogee!");
            current_state = LaunchState::steadyDescent;
            break;

        case LaunchState::steadyDescent :
            //will make method later, but
            current_state = LaunchState::Thousand_ft;
            break;

        case LaunchState::Thousand_ft :
            if(alt.getAltitude() == 1000) {
                // digitalWrite(igniter2, HIGH);
                Serial.println("Ignition 2 occured");
                }
            current_state = LaunchState::Descent;
            break;
        case LaunchState::Descent :
            // print sensor data?
            current_state = LaunchState::Touchdown;
            break;
        case LaunchState::Touchdown :
            //print gps readings
            break;
    }
}