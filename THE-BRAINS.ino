#include <AccelStepper.h>
float J1p, J2p, J3p;
float stepsPerRev;
float stepsPerDeg;

// CHANGE TO YOUR MICROSTEPPING RATIO. //
float ms = 0.25;


// ADD STEPPERS ///
AccelStepper J1(AccelStepper::DRIVER, 2, 3);
AccelStepper J2(AccelStepper::DRIVER, 4, 5);
AccelStepper J3(AccelStepper::DRIVER, 6, 7);

void setup() {
  // serial setup to communicate with python script
  Serial.begin(9600);

  /////MOTOR INITIALIZATION//////
  
  
  /*. This is where you set the maximum
  speed of the motors (in steps per second),
              and the acceleration.      */

  //J1
  J1.setMaxSpeed(1000);
  J1.setAcceleration(100);

  //J2
  J2.setMaxSpeed(1000);
  J2.setAcceleration(100);

  // J3
  J3.setMaxSpeed(1000);
  J3.setAcceleration(100);




  //MATH TO CONVERT ANGLES TO STEPS//
    stepsPerRev = 200 / ms;

    stepsPerRev = stepsPerRev * 20;

    stepsPerDeg = stepsPerRev / 360;

  
  
}

void loop() {
  // running steppers
  J1.run();
  J2.run();
  J3.run();
  
  // serial recieving from python
  if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        
        // split by comma
        int first  = data.indexOf(',');
        int second = data.indexOf(',', first + 1);
        
        J1p = data.substring(0, first).toFloat();
        J2p = data.substring(first + 1, second).toFloat();
        J3p = data.substring(second + 1).toFloat();
        
        // print the results
        Serial.println("got: " + String(J1p) + " " + String(J2p) + " " + String(J3p));
        //---- MOVE MOTORS WHEN SERIAL IS CHANGED -----//
        J1.moveTo(J1p * stepsPerDeg);
        J2.moveTo(J2p * stepsPerDeg);
        J3.moveTo(J3p * stepsPerDeg);
    }



}

