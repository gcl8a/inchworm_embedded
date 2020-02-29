#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"
#include "JointMotor2.h"

JointMotor2 jointMotor[3];
STATE state = ST_HOLDING;

// //Serial Buffer
// const int len = 16;
// char serialBuffer[len];
String inputBuffer; //String isn't the most efficient, but easier for I/O

void UpdateMotors(void);
void SetNewVias(void);
void StartMove(void);

void setup()
{
	Serial.begin(115200); //Debug Serial
	Wire.begin();		//begin I2C

	Serial.println("Robot intializing....");

	jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, 
		JOINT_MOTOR1_ADR, 10, 0, 0, 27.81, true, 0);
	jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, 
		JOINT_MOTOR2_ADR, 10, 0, 0, 124.38, true, 1);
	jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, 
		JOINT_MOTOR3_ADR, 10, 0, 0, 27.81, false, 2); 

	jointMotor[0].SetTarget(27.81);
	jointMotor[1].SetTarget(124.38);
	jointMotor[2].SetTarget(27.81);

	inputBuffer.reserve(24);

	Serial.println("Done");
}

uint32_t lastViaUpdate = 0;
uint32_t startMoveTime = 0;

void loop()
{
	if (Serial.available())
	{
		Serial.println("Message received");

		char c = Serial.read();
		inputBuffer += Serial.read();

		if(c == '\n')
		{
			if(inputBuffer[0] == 'M') 
			{
				StartMove();
			}

			if(inputBuffer[0] == 'P')
			{
				float k = inputBuffer.substring(1).toFloat();
				for(int i = 0; i < MOTOR_COUNT; i++)
				{
					jointMotor[i].SetKp(k);
				}
			}

			if(inputBuffer[0] == 'I')
			{
				float k = inputBuffer.substring(1).toFloat();
				for(int i = 0; i < MOTOR_COUNT; i++)
				{
					jointMotor[i].SetKi(k);
				}
			}

			if(inputBuffer[0] == 'D')
			{
				float k = inputBuffer.substring(1).toFloat();
				for(int i = 0; i < MOTOR_COUNT; i++)
				{
					jointMotor[i].SetKd(k);
				}
			}

			inputBuffer = "";
		}
	}

	static uint32_t lastUpdateTime = millis();
	uint32_t currTime = millis();
	if(currTime - lastUpdateTime >= UPDATE_INTERVAL)
	{
		if(currTime - lastUpdateTime >= UPDATE_INTERVAL)
			Serial.println("Missed update schedule.");

		lastUpdateTime += UPDATE_INTERVAL;

		if(state == ST_HOLDING || ST_MOVING)
		{
			UpdateMotors();
		}
	}

	if(currTime - lastViaUpdate >= VIA_INTERVAL)
	{
		if(state == ST_MOVING)
		{
			if(currTime - startMoveTime >= VIA_COUNT * VIA_INTERVAL)
			{
				state = ST_HOLDING;
			}

			else
				SetNewVias();
		}
	}
}

/*
* Update motor efforts
*/
void UpdateMotors()
{
	int speeds[MOTOR_COUNT];
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		speeds[i] = jointMotor[i].CalcEffort();
	}

	// jointMotor speed should be updated after all gcs are calculated to
	// minimize delay between each joint movement
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		jointMotor[i].SendPWM(speeds[i]);
	}
}

float startAngles[MOTOR_COUNT];
float targetAngles[MOTOR_COUNT];

void StartMove(void)
{
	for(int i = 0; i < MOTOR_COUNT; i++)
	{
		startAngles[i] = jointMotor[i].getAngleDegrees();
		targetAngles[i] = startAngles[i] + 10;
	}

	startMoveTime = millis();
	lastViaUpdate = startMoveTime;

	state = ST_MOVING;
}

void SetNewVias(void)
{
	uint32_t currTime = millis();
	float fraction = (currTime - startMoveTime) / (float)(VIA_COUNT * VIA_INTERVAL);
	if (fraction < 0) fraction = 0;
	if (fraction > 1) fraction = 1;

	for(int i = 0; i < MOTOR_COUNT; i++)
	{
		float viaAngle = startAngles[i] + (targetAngles[i] - startAngles[i]) * fraction;
		jointMotor[i].SetTarget(viaAngle);
	}
}