#include <Arduino.h>
#include "JointMotor2.h"

JointMotor2::JointMotor2() {
}

JointMotor2::JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double ang_offset, bool encoder_clockwise, int id_input) {
	//Pin Configuration
	pinDirectionA = pinDirectionA1;
	pinDirectionB = pinDirectionB1;
	pinPWM = pinPWM1;
	pinMode(pinDirectionA, OUTPUT);
	pinMode(pinDirectionB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
	//Encoder Setup
	encoder = AMS_AS5048B(encoderAddress);
	encoder.begin(); //Encoder Constructor
	encoder.setZeroReg(); //Zero Encoders
	//PID
	kP = kp;
	kI = ki;
	kD = kd;

	kP2 = kp;
	kI2 = ki;
	kD2 = kd;

	angle_offset = ang_offset;
	enc_clockwise = encoder_clockwise;
	encoder.setClockWise(enc_clockwise);
	lastPubAng=0;
	id=id_input;

	debug = false;
}
JointMotor2::JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double kp2, double ki2, double kd2, double ang_offset, bool encoder_clockwise, int id_input) {
	//Pin Configuration

	pinDirectionA = pinDirectionA1;
	pinDirectionB = pinDirectionB1;
	pinPWM = pinPWM1;
	pinMode(pinDirectionA, OUTPUT);
	pinMode(pinDirectionB, OUTPUT);
	pinMode(pinPWM, OUTPUT);

	//Encoder Setup
	encoder = AMS_AS5048B(encoderAddress);

	encoder.begin(); //Encoder Constructor

	encoder.setZeroReg(); //Zero Encoders


	//PID
	kP = kp;
	kI = ki;
	kD = kd;

	kP2 = kp2;
	kI2 = ki2;
	kD2 = kd2;

	//Variable to store both PID values  
    kP3 = kp;
    kI3 = ki;
    kD3 = kd; 

	angle_offset = ang_offset;
	enc_clockwise = encoder_clockwise;
	encoder.setClockWise(enc_clockwise);
	lastPubAng=0;
	id=id_input;

	debug = false;
}
/*
* Takes speed -255 - 255 and moves motor
*/
void JointMotor2::setSpeed(int speed) {
	double maxPercent = 0.75;
	if (speed < -255 * maxPercent) { speed = -255 * maxPercent; }
	else if (speed > 255 * maxPercent) { speed = 255 * maxPercent; }
	changeDirection(speed);
	analogWrite(pinPWM, abs(speed));
	return;
}
/*
* Changes motor direction pin states based on sign of speed
*/
void JointMotor2::changeDirection(int speed) {
	if (speed < 0) {
		digitalWrite(pinDirectionA, HIGH);
		digitalWrite(pinDirectionB, LOW);
	}
	else {
		digitalWrite(pinDirectionA, LOW);
		digitalWrite(pinDirectionB, HIGH);
	}
	return;
}
/*
* Gets encoder angle in degrees
*/
double JointMotor2::getAngleDegrees() {
	double angle = encoder.angleR(U_DEG, true);
	double calibrated_angle = 0;

	// if (debug) { Serial.print("angle: "); Serial.println(angle); }

	if (angle >= 0 && angle <= 360) { //don't return "I2C Error" as angle
		calibrated_angle = angle + angle_offset;
		if (calibrated_angle > 360) {
			calibrated_angle = calibrated_angle - 360;
		}
		last_calibrated_angle = calibrated_angle;
		if (debug) {
         if (millis()-lastPubAng>2000)
         {
           Serial.print("angle "); Serial.print(id); Serial.print(": "); Serial.println(calibrated_angle);
           lastPubAng=millis(); 
         }
         }
		return calibrated_angle;
	}
	else {
		return last_calibrated_angle;
	}
    
}
/*
* Set desired joint angle
*/
void JointMotor2::setAngle(double angle) {
	desiredAngle = angle;
	lastError = 0;
	sumError = 0;
	return;
}
/*
* Switch PID values for which joint is fixed
*/
void JointMotor2::switchPID(int gripperEngagedSelect){ 

    if(gripperEngagedSelect == 1){
        kP = kP2;
        kI = kI2;
        kD = kD2;
    }else if(gripperEngagedSelect == 2){
        kP = kP3;
        kI = kI3;
        kD = kD3;
    }

	Serial.println("In Switch PID");
}
/*
* Update motor speed for PID
*/
void JointMotor2::updateSpeed(int gc) {
	double currentAngle = getAngleDegrees();

	double error = desiredAngle - currentAngle;
	if (abs(int(error)) > 180) { error = error + 360; }

	sumError = sumError + error;
	double changeError = error - lastError;

	//int speed = (kP * error) + (kI * sumError) + (kD * changeError);
	
	int speed = gc;
	Serial.print("GC speed of angle "); 
	Serial.print(id); 
	Serial.print(": "); 
	Serial.println(speed);
           
	setSpeed(speed);

	// Serial.println(error);
	// Serial.println(sumError);
	// Serial.println(changeError);
	// Serial.println(kP);
	// Serial.println(kI);
	// Serial.println(kD);
	// Serial.println(speed);

	lastError = error;
	return;
}