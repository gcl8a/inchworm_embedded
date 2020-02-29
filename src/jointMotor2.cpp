#include "JointMotor2.h"
#include "config.h"

JointMotor2::JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, 
	uint8_t encoderAddress, double kp, double ki, double kd, 
	double ang_offset, bool encoder_clockwise, uint8_t id_input)
{
	//Pin Configuration
	pinDirectionA = pinDirectionA1;
	pinDirectionB = pinDirectionB1;
	pinPWM = pinPWM1;
	pinMode(pinDirectionA, OUTPUT);
	pinMode(pinDirectionB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
	//Encoder Setup
	encoder = AMS_AS5048B(encoderAddress);
	encoder.begin();	  //Encoder Constructor
	encoder.setZeroReg(); //Zero Encoders

	//PID
	kP = kp;
	kI = ki;
	kD = kd;

	angle_offset = ang_offset;
	enc_clockwise = encoder_clockwise;
	encoder.setClockWise(enc_clockwise);

	id = id_input;
}

const int maxDutyCycle = 230;
void JointMotor2::SendPWM(int speed)
{
	if (speed < 0)
	{
		if (speed < -maxDutyCycle) {speed = -maxDutyCycle;}
		digitalWrite(pinDirectionA, HIGH);
		digitalWrite(pinDirectionB, LOW);
	}
	else
	{
		if (speed > maxDutyCycle) {speed = maxDutyCycle;}
		digitalWrite(pinDirectionA, LOW);
		digitalWrite(pinDirectionB, HIGH);
	}

	analogWrite(pinPWM, abs(speed));
	return;
}

/*
* Gets encoder angle in degrees
*/
double JointMotor2::getAngleDegrees()
{
	double angle = encoder.angleR(U_DEG, true);
	double calibrated_angle = angle + angle_offset;

	if (calibrated_angle > 360) {calibrated_angle -= 360;}
	if (calibrated_angle <   0) {calibrated_angle += 360;}

	double delta = calibrated_angle - last_calibrated_angle;
	if(delta > 180) delta -= 360;
	if(delta < -180) delta += 360;

	if(fabs(delta) > ANGLE_ERROR_THRESHOLD)
	{
		Serial.println("Angle error!");
	}

	last_calibrated_angle = calibrated_angle;
	return calibrated_angle;
}

/*
* Set desired joint angle
*/
void JointMotor2::SetTarget(double angle)
{
	targetAngle = angle;
	return;
}

int JointMotor2::CalcEffort(void)
{
	double currentAngle = getAngleDegrees();

	double error = targetAngle - currentAngle;

	if(error > 180) error -= 360;
	if(error < -180) error += 360;

	sumError += error;

	//Cap sum of the error
	if (sumError > SUM_THRESHOLD)
	{
		sumError = SUM_THRESHOLD;
	}
	if (sumError < -SUM_THRESHOLD)
	{
		sumError = -SUM_THRESHOLD;
	}
	
	double deltaError = error - lastError;

	double effort = (kP * error) + (kI * sumError) + (kD * deltaError);

	lastError = error;

	return effort;
}
