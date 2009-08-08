#include "Nemala.h"


#define CALIB_DIV 1.1578
#define CALIB_TOL 0
#define CALIB_ERRTIMES 20
#define CALIB_DRIFTLIMIT 20

typedef enum {REGULAR, Joystick} NemalaMode;
int main(int argc, char *argv[])
{
	try{
		Nemala nemala;
		int whattodo;
		int param;
		bool fContinue = true;
		NemalaMode mode = REGULAR;

		while (fContinue) {
			if(mode == REGULAR)
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
				cout << "8. Go in straight line" << endl;
				cout << "9. Reset Drift" << endl;
				cout << "-1. TERMINATE!" << endl;
				cout << "99. Move to joystick mode" << endl;
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
							while(1) {
						   param = nemala.readSonar(0);
						   cout << "Sonar: " << param << endl;;
							}
						   break;
						case 8: 
							int err_count; // after CALIB_ERRTIMES it will give a push to the other side
							int glob_calib_fix;
							err_count=0;
							glob_calib_fix=0;
							nemala.setMaxSpeed(100);
							while(1) {
								int left, right;
								nemala.driveForward();
							    left = nemala.getLeftEncoder()+glob_calib_fix;
							    right = nemala.getRightEncoder();
							    if (left > right+CALIB_TOL) {
									err_count+=1;
									if (err_count > CALIB_ERRTIMES) {
										glob_calib_fix=left-right;
										left -= glob_calib_fix;
										err_count=0;
										nemala.zeroEncoders();
									}
									if ((left-right)/CALIB_DIV > CALIB_DRIFTLIMIT) {
										nemala.setDriftSpeed(CALIB_DRIFTLIMIT);
									} else {
										nemala.setDriftSpeed((left-right)/CALIB_DIV);
									}
									nemala.setDriftDirection(RIGHT);
							    } else if (right > left+CALIB_TOL) {
									err_count-=1;
									if (err_count < -CALIB_ERRTIMES) {
										glob_calib_fix=left-right;
										left += glob_calib_fix;
										err_count=0;
										nemala.zeroEncoders();
									}
									if ((right-left)/CALIB_DIV > CALIB_DRIFTLIMIT) {
										nemala.setDriftSpeed(CALIB_DRIFTLIMIT);
									} else {
										nemala.setDriftSpeed((right-left)/CALIB_DIV);
									}
									nemala.setDriftDirection(LEFT);
								} else {
									nemala.setDriftSpeed(0);
									nemala.setDriftDirection(LEFT);
								}
								cout << "Left: " << left << " Right: " << right << " Diff: " << left-right << endl;
							}
						   break;
					   case 9: 
							nemala.setDriftSpeed(0);
							nemala.setDriftDirection(LEFT);
							nemala.driveForward();
							break;
					   case 99: 
						   mode = Joystick;
						   cout << "Joystick mode" << endl;
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