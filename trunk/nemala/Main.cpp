#include "Nemala.h"
#include <windows.h>

#define CALIB_DIV 1.1578
#define CALIB_TOL 0
#define CALIB_ERRTIMES 2999
//#define CALIB_DRIFTLIMIT 20
//#define MM_PER_ENC_TICK 1.7640573318632855567805953693495

/************************************************************************/
/* Robot moving mode                                                    */
/************************************************************************/
typedef enum {REGULAR, Joystick, Automatic, Bug} NemalaMode;
int main(int argc, char *argv[])
{
	try{
		Nemala nemala(new Map(POINT_3_X, POINT_3_Y, POINT_1_X, POINT_1_Y), WEST);
		int whattodo;
		int param;
		bool fContinue = true;
		NemalaMode mode = REGULAR;
		while (fContinue) {
			if(mode == Bug)
			{
				int currX, currY, nextX, nextY;
				nemala.calibrate();
				system("PAUSE");
				/* Set the robot to the right orientation */
				//nemala.firstFineTune();

				/* For each two following stations calculate the path and drive\turn the robot accordingly */
				StationType currSt,  nextSt;
				BUG_STATE bs;
				currSt = nemala.map->getNextStation(currX, currY);
				cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << currX << "," << currY << ")" << " Type: " << currSt << endl;
				while( (nextSt = nemala.map->getNextStation(nextX, nextY)) != NOT_STATION)
				{
					if(currX == nextX) /* | */
					{
						bs = nemala.driveYaxis_BUG(currY, nextY, currX, nextSt);
						if(bs == FREE)
							nemala.map->fillYaxis(currY, nextY,	currX, RED);
						else 
							nemala.map->fillYaxis(currY, nemala.curr_y, currX, RED);
					}
					else if(currY == nextY) /* - */
					{
						bs = nemala.driveXaxis_BUG(currX, nextX, currY, nextSt);
						if(bs == FREE)
							nemala.map->fillXaxis(currX, nextX, currY, RED);
						else
							nemala.map->fillXaxis(currX, nemala.curr_x, currY, RED);
					}
					else
						throw "Exception: Koo3";

					if(bs == BLOCKED)
					{	
						OBS_POS op = nemala.bypass();
						currX = nemala.curr_x;
						currY = nemala.curr_y;
						if(nemala.map->updatePath(currX, currY))
							continue;
						else
						{
							nemala.bypass2ndChance(op);
							currX = nemala.curr_x;
							currY = nemala.curr_y;
							if(nemala.map->updatePath(currX, currY))
								continue;
							else
								throw "Could not update path!";
						}
					}
					cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << nextX << "," << nextY << ")" << " Type: " << nextSt << endl;
					cout << "X: " << nemala.curr_x << " Y: " << nemala.curr_y << endl;
					currX  = nextX;
					currY  = nextY;
					currSt = nextSt;
				}
				nemala.lastFineTune();
				system("PAUSE");
				return 0;
			}
			else if(mode == Automatic)
			{
				int currX, currY, nextX, nextY;
				nemala.calibrate();
				system("PAUSE");
				/* Set the robot to the right orientation */
				nemala.firstFineTune();

				/* For each two following stations calculate the path and drive\turn the robot accordingly */
				StationType currSt,  nextSt;
				currSt = nemala.map->getNextStation(currX, currY);
				cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << currX << "," << currY << ")" << " Type: " << currSt << endl;
				while( (nextSt = nemala.map->getNextStation(nextX, nextY)) != NOT_STATION)
				{
					if(currX == nextX) /* | */
						nemala.driveYaxis(currY, nextY, currX, nextSt);
					else if(currY == nextY) /* - */
						nemala.driveXaxis(currX, nextX, currY, nextSt);
					else
						throw "Exception: Koo3";
					currX  = nextX;
					currY  = nextY;
					currSt = nextSt;
					cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << nextX << "," << nextY << ")" << " Type: " << nextSt << endl;
				}
				nemala.lastFineTune();
				return 0;
			}
			else if(mode == REGULAR)
			{
				bool status;
				bool gstatus;
				cout << "What do you want to do?" << endl;
				cout << "1. Drive forward" << endl;
				cout << "2. Drive Backward" << endl;
				cout << "30. TURN Left" << endl;
				cout << "31. TURN Right" << endl;
				cout << "4. STOP" << endl;
				cout << "50. Get Maximum Speed" << endl;
				cout << "51. Get Minimum Speed" << endl;
				cout << "52. Get Drift Speed" << endl;
				cout << "53. Get Drift Direction" << endl;
				cout << "60. Get Left Encoder" << endl;
				cout << "61. Get Right Encoder" << endl;
				cout << "70. Get Sonar 0" << endl;
				cout << "71. Get Sonar 1" << endl;
				cout << "72. Get Sonar 2" << endl;
				cout << "73. Get Sonar 3" << endl;
				cout << "80. Go in straight line for x mms" << endl;
				cout << "81. Turn 90degrees right" << endl;
				cout << "9. Reset Drift" << endl;
				cout << "-1. TERMINATE!" << endl;
				cout << "99. Move to joystick mode" << endl;
				cout << "98. Move to Automatic mode" << endl;
				cout << "97. Move to BUG mode" << endl;
				cout << "z. Return to normal mode" << endl;
				cin >> whattodo;

				status = false;
				gstatus = false;
				switch (whattodo) {
					   case -1: 
						   nemala.terminate();
						   fContinue = false;
						   break;
					   case 1: 
						   nemala.driveForward();
						   break;
					   case 2: 
						   nemala.driveBackward();
						   break;
					   case 30: 
						   nemala.turnLeft();
						   break;
					   case 31: 
						   nemala.turnRight();
						   break;
					   case 4: 
						   nemala.stop();
						   break;
					   case 50: 
						   param = nemala.getMaxSpeed();
						   cout << "MaxSpeed: " << param << endl;
						   break;
					   case 51: 
						   param = nemala.getMinSpeed();
						   cout << "MinSpeed: " << param << endl;
						   break;
					   case 52: 
						   param = nemala.getDriftSpeed();
						   cout << "DriftSpeed: " << param << endl;
						   break;
					   case 53: 
						   param = nemala.getDriftDirection();
						   cout << "DriftDirection: " << param << endl;
						   break;
					   case 60: 
						   param = nemala.getLeftEncoder();
						   cout << "LeftEncoder: " << param << endl;;
						   break;
					   case 61: 
						   param = nemala.getRightEncoder();
						   cout << "RightEncoder: " << param << endl;;
						   break;
						case 70: 
							for (int i=0; i<100; i++) {
						   param = nemala.readSonar(0);
						   cout << "Sonar: " << param << endl;;
							}
						   break;
						   case 72: 
							while(1) {
						   param = nemala.readSonar(2);
						   cout << "Sonar: " << param << endl;;
							}
						   break;
						   case 74: 
							for (int i=0; i<100; i++) {
						   param = nemala.readSonar(4);
						   cout << "Sonar: " << param << endl;;
							}
						   break;
						case 80: 
							int dist;
							cin >> dist;
							nemala.driveForward(dist,-1,-1,-1);
						   break;
						case 81:
							float amount;
							cin >> amount;
							nemala.turnRight(amount);
							break;
					    case 82:
							cin >> amount;
							nemala.turnLeft(amount);
							break;
						case 8000:
							short left, right;
							nemala.zeroEncoders();
							while (1) {
								nemala.turnLeft(0x20);
								for (int i=0; i<100; i++) {
									left = nemala.getLeftEncoder();
									right = nemala.getRightEncoder();
									cout << "Right: " << right << " Left: " << left << endl;
								}
								nemala.turnRight(0x20);
								for (int i=0; i<100; i++) {
									left = nemala.getLeftEncoder();
									right = nemala.getRightEncoder();
									cout << "Right: " << right << " Left: " << left << endl;
								}
							}
					   case 9: 
							nemala.setDriftSpeed(0);
							nemala.setDriftDirection(LEFT);
							nemala.driveForward();
							break;
					   case 98: 
						   mode = Automatic;
						   cout << "Automatic mode" << endl;
						   break;
					   case 97: 
						   mode = Bug;
						   cout << "Bug mode" << endl;
						   break;
					   case 99: 
						   mode = Joystick;
						   cout << "Joystick mode" << endl;
						   break;
					   case 101:
						   nemala.calibrate();
						   cout << nemala.glob_calib_avg << endl;
						   cout << nemala.glob_calib_dir << endl;
						   break;
					   case 100:
						   nemala.turnLeft2(0.25);
						   break;
						case 1000:
						   //nemala.driveForward(800);
						   //nemala.turnLeft(0.25);
						   //nemala.driveForward(2550);
						   //nemala.turnLeft(0.25);
						   //nemala.driveForward(800);
							
							/* test for the hardcoded with first and last finetune
							*/
							//nemala.firstFineTune();
							//nemala.driveForward(1100,30,-1,-1);
							//nemala.turnLeft(0.25);
							//nemala.driveForward(900);
							//nemala.driveForward(540);
							//nemala.driveForward(540);
							//nemala.driveForward(460,-1,-1,30);
							//nemala.turnRight(0.25);
							//nemala.driveForward(11110,30,-1,30);
							//nemala.curr_o=WEST;
							//nemala.curr_x=30;
							//nemala.curr_y=285;
							//nemala.lastFineTune();
							

							// test for the byPass (bug)
							nemala.calibrate();
							system("PAUSE");
							OBS_POS p = nemala.leftByPass();
							BUG_STATE state = nemala.secondByPass(p);
							system("PAUSE");
							//nemala.firstFineTune();
							/*
							if (nemala.driveForwardBug(1100,30,-1,20) == BLOCKED) {
								nemala.leftByPass();
							}
							nemala.turnLeft(0.25);
							if (nemala.driveForwardBug(900,-1,-1,20) == BLOCKED) {
								nemala.leftByPass();
							}
							if (nemala.driveForwardBug(540,-1,-1,20) == BLOCKED) {
								nemala.leftByPass();
							}
							if (nemala.driveForwardBug(540,-1,-1,20) == BLOCKED) {
								nemala.leftByPass();
							}
							if (nemala.driveForwardBug(460,-1,-1,20) == BLOCKED) {
								nemala.leftByPass();
							}
							nemala.turnLeft(0.25);
							nemala.driveForwardBug(11110,30,-1,30);
							nemala.lastFineTune();
							*/
							//nemala.turnLeft();
							//nemala.driveForward(520,30, 84);
						   break;
				}
			}
			else if(mode == Joystick)
			{
				char move = getch();
				if(move == 'z')
				{
					mode = REGULAR;
					cout << "Regular mode" << endl;
				}
				else if(move == 32)
				{
					nemala.stop();
				}
				else if(move == 72)
				{
					nemala.driveForward();
				}
				else if(move == 80)
				{
					nemala.driveBackward();
				}
				else if(move == 77)
				{
					nemala.turnRight();
				}
				else if(move == 75)
				{
					nemala.turnLeft();
				}
			}
		}
	}
	catch (NemalaException ne)
	{
		cout << "NemalaException: " << ne << endl;
	}
	system("PAUSE");
	return 0;
}