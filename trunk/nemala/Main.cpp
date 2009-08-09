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
				cout << "80. Go in straight line for x mms" << endl;
				cout << "81. Turn 90degrees right" << endl;
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
						   case 72: 
							while(1) {
						   param = nemala.readSonar(2);
						   cout << "Sonar: " << param << endl;;
							}
						   break;
						   case 74: 
							while(1) {
						   param = nemala.readSonar(4);
						   cout << "Sonar: " << param << endl;;
							}
						   break;
						case 80: 
							int err_count; // after CALIB_ERRTIMES it will give a push to the other side
							int glob_calib_fix;
							int glob_r_enc, glob_l_enc;
							int howlong;
							cin >> howlong;
							glob_r_enc=0;
							glob_l_enc=0;
							err_count=0;
							glob_calib_fix=nemala.getDriftSpeed()*CALIB_DIV;
							nemala.zeroEncoders();
							while(1) {
								short left, right;
								nemala.driveForward();
							    left = nemala.getLeftEncoder();
							    right = nemala.getRightEncoder();
								glob_r_enc=right;
								glob_l_enc=left;
								//left+=glob_calib_fix;
								glob_calib_fix=left-right;
								//nemala.zeroEncoders();
							    if (left > right+CALIB_TOL) {
									err_count+=1;
									if (err_count > CALIB_ERRTIMES) {
										left -= glob_calib_fix;
										err_count=0;
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
										left += glob_calib_fix;
										err_count=0;
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
								cout << "Left: " << glob_r_enc << " Right: " << glob_l_enc << " Diff: " << left-right << " and: " << glob_l_enc-glob_r_enc << endl;
								if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
									nemala.stop();
									break;
								}
							}
						   break;
						case 81:
							short left, right;
							int drift;
							int turn_amount;
							//int turn_speed;
							//turn_speed = 0x00;
							turn_amount=0.25*TICKS_PER_360_DEG;
							right=left=0;
							nemala.zeroEncoders();
							glob_r_enc=0;
							glob_l_enc=0;
							//nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
							while (1) {
								left = nemala.getLeftEncoder()+glob_l_enc;
								right = nemala.getRightEncoder()+glob_r_enc;
								glob_r_enc=right;
								glob_l_enc=left;
								if ((left+right) < turn_amount-TURN_TOLERANCE) {
									nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
									nemala.setDriftDirection(RIGHT);
									//nemala.setDriftSpeed(0);
									nemala.zeroEncoders();
									nemala.turnRight(0x20);
									nemala.stop();
								} else if ((left+right) > turn_amount+TURN_TOLERANCE) {
									while ((left+right) > turn_amount+TURN_TOLERANCE) {
										nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
										nemala.setDriftDirection(RIGHT);
										nemala.zeroEncoders();
										nemala.turnLeft(0x02);
										nemala.stop();
										left = glob_l_enc-nemala.getLeftEncoder();
										right = glob_r_enc-nemala.getRightEncoder();
										glob_r_enc=right;
										glob_l_enc=left;
										cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
									}
									nemala.setDriftSpeed(0);
									nemala.stop();
									break;
								} else {
									nemala.setDriftSpeed(0);
									nemala.stop();
									cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
									break;
								}
								cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
							}
							//cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
							nemala.setDriftSpeed(0);
							nemala.stop();
						   break;
					    case 82:
							//int turn_speed;
							//turn_speed = 0x00;
							turn_amount=0.25*TICKS_PER_360_DEG;
							right=left=0;
							nemala.zeroEncoders();
							glob_r_enc=0;
							glob_l_enc=0;
							//nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
							while (1) {
								left = nemala.getLeftEncoder()+glob_l_enc;
								right = nemala.getRightEncoder()+glob_r_enc;
								glob_r_enc=right;
								glob_l_enc=left;
								if ((left+right) < turn_amount-TURN_TOLERANCE) {
									nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
									nemala.setDriftDirection(RIGHT);
									//nemala.setDriftSpeed(0);
									nemala.zeroEncoders();
									nemala.turnLeft(0x20);
									nemala.stop();
								} else if ((left+right) > turn_amount+TURN_TOLERANCE) {
									while ((left+right) > turn_amount+TURN_TOLERANCE) {
										nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
										nemala.setDriftDirection(RIGHT);
										nemala.zeroEncoders();
										nemala.turnRight(0x02);
										nemala.stop();
										left = glob_l_enc-nemala.getLeftEncoder();
										right = glob_r_enc-nemala.getRightEncoder();
										glob_r_enc=right;
										glob_l_enc=left;
										cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
									}
									nemala.setDriftSpeed(0);
									nemala.stop();
									break;
								} else {
									nemala.setDriftSpeed(0);
									nemala.stop();
									cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
									break;
								}
								cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
							}
							nemala.setDriftSpeed(0);
							//cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
							nemala.stop();
						   break;
						case 8000:
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