#include "Nemala.h"
#include <windows.h>

#define CALIB_DIV 1.1578
#define CALIB_TOL 0
#define CALIB_ERRTIMES 2999
#define CALIB_DRIFTLIMIT 20
#define MM_PER_ENC_TICK 1.7640573318632855567805953693495
#define MM_PER_360_DEG 200
#define TICKS_PER_360_DEG 316
#define TURN_TOLERANCE 0

typedef enum {REGULAR, Joystick, Automatic} NemalaMode;
int main(int argc, char *argv[])
{
	try{
		Nemala nemala;
		int whattodo;
		int param;
		bool fContinue = true;
		NemalaMode mode = Automatic;

		while (fContinue) {
			if(mode == REGULAR)
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
						   right_distance=nemala.readSonar(4);
							left_distance = nemala.readSonar(0);
							front_distance = nemala.readSonar(2);
						   while (!(((right_distance <= 35) && (right_distance >= 25)) && ((left_distance <= 105) && (left_distance >= 95)))) {
							   nemala.turnLeft();
							right_distance=nemala.readSonar(4);
							left_distance = nemala.readSonar(0);
							front_distance = nemala.readSonar(2);
							cout << "Right: " << right_distance << " Left: " << left_distance << " Front: " << front_distance << endl;
							}
						   break;
						case 1000:
						   //nemala.driveForward(800);
						   //nemala.turnLeft(0.25);
						   //nemala.driveForward(2550);
						   //nemala.turnLeft(0.25);
						   //nemala.driveForward(800);
							nemala.driveForward(540,30);
							nemala.turnLeft(0.25);
							nemala.driveForward(1100,-1,84);
							nemala.driveForward(520,30, 30);
							nemala.driveForward(520,30, 84);
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
			else if(mode == Automatic)
			{
				int currX, currY, nextX, nextY;
				int northDist, southDist, eastDist , westDist;
				
				//nemala.map->print();

				nemala.map->getNextStation(currX, currY);
				cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << currX << "," << currY << ")" << endl;

				while(nemala.map->getNextStation(nextX, nextY))
				{
					cout << "Station " << nemala.map->getCurrStation() << ": " << "(" << nextX << "," << nextY << ")" << endl;
					if(currX == nextX)
					{/* _ */

					}
					else if(currY == nextY)
					{/* | */

					}
					else /* Choose path: (1) |_ (2) _| */
					{
						int distXaxis = nemala.map->getDistance(nextX, currY),
							distYaxis = nemala.map->getDistance(currX, nextY);

						if(distXaxis < distYaxis) /* |_ */
						{

						}
						else if(distYaxis < distXaxis) /* _| */
						{

						}
					}
					currX = nextX;
					currY = nextY;
				}
				return 0;
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