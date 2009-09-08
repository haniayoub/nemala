#include "Nemala.h"
#include <windows.h>

#define CALIB_DIV 1.1578
#define CALIB_TOL 0
#define CALIB_ERRTIMES 2999
//#define CALIB_DRIFTLIMIT 20
#define MM_PER_ENC_TICK 1.7640573318632855567805953693495
#define MM_PER_360_DEG 200

/************************************************************************/
/* Robot moving mode                                                    */
/************************************************************************/
typedef enum {REGULAR, Joystick, Automatic} NemalaMode;
int main(int argc, char *argv[])
{
	try{
		Nemala nemala(new Map(POINT_1_X, POINT_1_Y, POINT_4_X, POINT_4_Y), WEST);
		int whattodo;
		int param;
		bool fContinue = true;
		NemalaMode mode = REGULAR;

		while (fContinue) {
			if(mode == Automatic)
			{
				int currX, currY, nextX, nextY;

				/* Set the robot to the right orientation */
				nemala.fineTune();

				/* For each two following stations calculate the path and drive\turn the robot accordingly */
				nemala.map->getNextStation(currX, currY);
				cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << currX << "," << currY << ")" << endl;
				while(nemala.map->getNextStation(nextX, nextY))
				{
					if(currX == nextX)
					{ /* | */
						nemala.driveYaxis(currY, nextY);
					}
					else if(currY == nextY)
					{ /* - */
						nemala.driveYaxis(currY, nextY);
					}
					else /* Choose path: (1) |_ (2) _| */
					{
						int distXaxis = nemala.map->getDistance(nextX, currY),
							distYaxis = nemala.map->getDistance(currX, nextY);

						if(distXaxis > distYaxis) /* _| */
						{
							nemala.driveXaxis(currX, nextX);
							nemala.driveYaxis(currY, nextY);
						}
						else if(distYaxis > distXaxis) /* |_ */
						{
							nemala.driveYaxis(currY, nextY);
							nemala.driveXaxis(currX, nextX);							
						}
					}
					currX = nextX;
					currY = nextY;
					cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << nextX << "," << nextY << ")" << endl;
				}
				return 0;
			}			
			else if(mode == REGULAR)
			{
				bool status;
				bool gstatus;
				Distance right_distance, left_distance, front_distance;
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
							nemala.driveForward(dist);
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
							nemala.driveForward(1160,-30);
							nemala.turnLeft(0.25);
							nemala.driveForward(900);
							nemala.driveForward(540);
							nemala.driveForward(540);
							nemala.driveForward(460);
							nemala.turnLeft(0.25);
							nemala.driveForward(1110,30);
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