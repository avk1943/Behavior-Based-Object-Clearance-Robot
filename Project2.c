/*
 CSE 4360 - Project 2
 Name: Andy Sustaita, Kierra Thompson, Ashwitha Kassetty
*/

#include <ev3.h>
#include <time.h>
#include <stdlib.h>

#define MIN_WALL_DIST 50
#define MAX_WALL_DIST 60
#define MEAN_WALL_DIST (MIN_WALL_DIST + MAX_WALL_DIST)/2
#define SPEED 20
#define DEVIATION_FACTOR 2
#define TURN_ANGLE 183
#define K_V 0.08
#define FWD_ANGLE 350


#define WANDER 1
#define FINDWALL 2
#define FOLLOWWALL 3
#define MOVETOWALL 4
#define MOVEALONGWALL 5

int at_goal = 0;
int curr_distance, deviation, last_distance;
int speed_offset_a = 0, speed_offset_d = 0;
int error_A = 0;
int error_D = 0;

//Left wheel - A
//Right wheel - D
//Light Sensor - IN_1
//Sonar Sensor - IN_4
//Blue is 2 from light sensor
//Red is 5 from light sensor

/*The move_forward function moves the robot forward*/
void move_forward()
{
	int initialCountA = MotorRotationCount(OUT_A) + error_A;
	int initialCountD = MotorRotationCount(OUT_D) + error_D;
	while((((FWD_ANGLE - MotorRotationCount(OUT_A) - initialCountA) > 10) && (FWD_ANGLE - MotorRotationCount(OUT_D) - initialCountD) > 10))
	{
		OnFwdSync(OUT_AD, 0.04*(FWD_ANGLE - abs(MotorRotationCount(OUT_A) - initialCountA)) + 3);
		Wait(100);

	}

	RotateMotor(OUT_A, 3, (FWD_ANGLE - abs(MotorRotationCount(OUT_A) - initialCountA))- 5);
	RotateMotor(OUT_D, 3, (FWD_ANGLE - abs(MotorRotationCount(OUT_D) - initialCountD))- 5);
}

/*The left_turn_90 function turns the robot 90 degrees to the left along the center of the robot*/
int left_turn_90(int action)
{
		action = 2;
        int currentCountA = MotorRotationCount(OUT_A) + error_A;
        int currentCountD = MotorRotationCount(OUT_D) + error_D;

       OnFwdReg(OUT_A, K_V*(TURN_ANGLE - (MotorRotationCount(OUT_A) - currentCountA)) + 3);
       OnFwdReg(OUT_D, K_V*(TURN_ANGLE - (MotorRotationCount(OUT_D) - currentCountD)) + 3);

       OnFwdReg(OUT_A, 4);
       OnFwdReg(OUT_D, -4);
       Wait(500);
       return action;
}

/*The right_turn_90 function turns the robot 90 degrees to the right along the center of the robot*/
int right_turn_90(int action)
{
		action = 2;
        int currentCountD = MotorRotationCount(OUT_D) + error_D;
        int currentCountA = MotorRotationCount(OUT_A) + error_A;

       OnFwdReg(OUT_D, K_V*(TURN_ANGLE - (MotorRotationCount(OUT_D) - currentCountD)) + 3);
       OnFwdReg(OUT_A, K_V*(TURN_ANGLE - (MotorRotationCount(OUT_A) - currentCountA)) + 3);

       OnFwdReg(OUT_D, 4);
       OnFwdReg(OUT_A, -4);
       Wait(500);
       return action;
}

/*The detect function detects/reads the color from the light sensor and returns the value/color detected*/
int detect()
{
	int color;
	color = ReadSensor(IN_1);
    return color;
}

/*The move_away_wall function moves the robot backwards and away from the wall once it detects a wall (blue tape)*/
void move_away_wall()
{
    deviation = MIN_WALL_DIST - curr_distance;
    OnFwdSync(OUT_AD, -15);
    Wait(1000);
    Off(OUT_AD);
    return;
}

/* The Clearing function is called when the robot detects the goal area (red tape). The robot moves forward to push the can out of the goal*/
void Clearing ()
{
    TermPrintf("Detected Goal - Moving Forward to push the can out of the red tape!\n");
    Wait(500);
	ResetRotationCount(OUT_A);
	ResetRotationCount(OUT_D);
    move_forward();
    return;

}

/*
The Goal_finding function takes in the action variable. The robot will rotate in a complete circle until it detects the can within a foot from it. It will return the number associated with the turn right function.
*/
int Goal_finding (int action)
{
    int i;
    int c;

    for(i = 0; i < 4; i++)
    {
    	ResetRotationCount(OUT_A);
    	ResetRotationCount(OUT_D);
    	c = ReadSensor(IN_4);
        if(c < 310)
        	{
        		return action;
        	}
        action = right_turn_90(action);
    }
    return action;
}

/*
The Wander function takes in an int variable that keeps track of the previous action taken. Inside the function a random number will be generated to which we take the modulus of 6 from it. We prioritized moving forward over turning. If we moved away from a wall (action == 5), then we prioritize turning over moving forward.
*/
int Wander (int action)
{
	srand(time(0));
    int c = rand();
    int choice = c % 6;
    if(choice == 0 ||choice == 2||choice == 4||choice == 5)
    {
    	if(action != 5)
    	{
			if(detect() == 2)
			{
				goto skip;
			}
			ResetRotationCount(OUT_A);
			ResetRotationCount(OUT_D);
			while(detect()!=2)
			{
				OnFwdSync(OUT_AD, 20);
			}
			if(detect() == 2)
			{
				Off(OUT_AD);
			}
			skip:
			Wait(1000);
			Off(OUT_AD);
			action = 0;
			return action;
    	}
    }
    else if(choice == 1)
    {
    	ResetRotationCount(OUT_A);
    	ResetRotationCount(OUT_D);
        action = left_turn_90(action);
        Wait(100);
        ResetRotationCount(OUT_A);
        ResetRotationCount(OUT_D);
        action = left_turn_90(action);
        Wait(100);
        return action;
    }
    else if(choice == 3)
    {
    	ResetRotationCount(OUT_A);
    	ResetRotationCount(OUT_D);
    	action = right_turn_90(action);
        Wait(500);
        ResetRotationCount(OUT_A);
        ResetRotationCount(OUT_D);
        action = right_turn_90(action);
        Wait(500);
        return action;
    }
    return action;
}

/*
The Wall_following function is the overall robot traversal behavior. We create a integer variable that keeps track of the previous action. While inside the function, the robot will continuously wander unless it hits the wall or reaches the goal. If the robot hits the wall, it will move away. If the goal is detected, then the robot will position itself in front of the can and push it off the goal location.
*/
void Wall_following ()
{
	int action = 0;
	int catch;
	while(!at_goal)
	{
		if(detect()==5)
		{
			at_goal = 1;
			Goal_finding(action);
			PlayTone(TONE_C2, 500);
			Clearing();
			TermPrintln("!!!At goal!!!");
			Wait(10);
			return;
		}
		else if(detect() == 2)
		{
			action = 5;
			move_away_wall();
		}
		else
		{
			catch = Wander(action);
			action = catch;
		}
		Wait(100);
	}
}

int main(void)
{
	InitEV3();
	SetAllSensorMode(COL_COLOR, NO_SEN, NO_SEN, US_DIST_MM);
	Wall_following();
	TermPrintf("Press ENTER to exit");
	ButtonWaitForPress(BUTTON_ID_ENTER);
	FreeEV3();
	return 0;
}
