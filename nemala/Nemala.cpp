#include "Nemala.h"

// defines
//#define STRICT
//#define DEBUG
#define COMPORT "COM2"
#define CHECK_SUM(x,y,z) ((char)(((char)x+(char)y+(char)z)%256))
#define BUFF_SIZE 5
#define DEFAULT_SPEED 30
#define SonarForCM 256
#define SonarDFEpselon 1
//#define TICKS_PER_360_DEG 316
//#define TICKS_PER_360_DEG 184
#define TICKS_PER_360_DEG 246
#define CALIB_DRIFTLIMIT 10
#define TURN_TOLERANCE 0
#define MM_BETWEEN_SONAR_READS 250
#define SONAR_NR_OF_FIXES 8
#define FINAL_DEST_EPSELON 5

int getMid(int x, int y, int z) {
	if (((x <= y) && (y<=z)) || ((x >= y) && (y>=z))) return y;
	if (((y <= x) && (x<=z)) || ((y >= x) && (x>=z))) return x;
	if (((x <= z) && (z<=y)) || ((x >= z) && (z>=y))) return z;
}

Nemala::Nemala(Map *map, Orientation o)
{
#ifndef DEBUG
	_connect();
	zeroEncoders();
#endif
	this->glob_calib_avg=0;
	this->map = map;
	this->curr_o = o;
}

Nemala::~Nemala()
{
	_disconnect();
}

Orientation Nemala::figureMyOrientation(int my_front, int my_left, int my_right, int needed_front, int needed_left, int needed_right, int needed_back, Orientation needed_o) {
	Orientation o;
	int north_err, south_err, east_err, west_err;
	north_err = abs(my_front-needed_front)+abs(my_left-needed_left)+abs(my_right-needed_right);
	south_err = abs(my_front-needed_back)+abs(my_left-needed_right)+abs(my_right-needed_left);
	east_err = abs(my_front-needed_left)+abs(my_left-needed_back)+abs(my_right-needed_front);
	west_err = abs(my_front-needed_right)+abs(my_left-needed_front)+abs(my_right-needed_back);
	int minimum = min(min(north_err,south_err),min(east_err,west_err));
	if (minimum == north_err) o=NORTH;
	if (minimum == south_err) o=SOUTH;
	if (minimum == east_err) o=EAST;
	if (minimum == west_err) o=WEST;
	cout << "Errors North: " << north_err <<  " South: " << south_err <<  " East: " << east_err <<  " west: " << west_err << endl;
	if (needed_o==NORTH) {
		switch ((int)o) {
			case NORTH:
				return NORTH;
				break;
			case SOUTH:
				return SOUTH;
				break;
			case EAST:
				return EAST;
				break;
			case WEST:
				return WEST;
				break;
		}
	}
	if (needed_o==WEST) {
		switch ((int)o) {
			case NORTH:
				return WEST;
				break;
			case SOUTH:
				return EAST;
				break;
			case EAST:
				return SOUTH;
				break;
			case WEST:
				return NORTH;
				break;
		}
	}
	if (needed_o==EAST) {
		switch ((int)o) {
			case NORTH:
				return EAST;
				break;
			case SOUTH:
				return WEST;
				break;
			case EAST:
				return NORTH;
				break;
			case WEST:
				return SOUTH;
				break;
		}
	}
	if (needed_o==SOUTH) {
		switch ((int)o) {
			case NORTH:
				return SOUTH;
				break;
			case SOUTH:
				return NORTH;
				break;
			case EAST:
				return WEST;
				break;
			case WEST:
				return EAST;
				break;
		}
	}
}

void Nemala::firstFineTune() {
	int real_front_dist, real_back_dist, real_right_dist, real_left_dist;
	int front_dist, right_dist, left_dist;
	map->getDistances(map->src_x, map->src_y, curr_o, real_front_dist, real_back_dist, real_right_dist, real_left_dist);
	front_dist=getMid(readSonar(2),readSonar(2),readSonar(2));
	left_dist=getMid(readSonar(0),readSonar(0),readSonar(0));
	right_dist=getMid(readSonar(4),readSonar(4),readSonar(4));
	cout << "Front: " << front_dist << " left: " << left_dist <<  " right: " << right_dist  << endl;
	cout << "Needed Front: " << real_front_dist << " back: " << real_back_dist << " left: " << real_left_dist <<  " right: " << real_right_dist  << endl;
	Orientation needed_to_be = curr_o;
	curr_o = figureMyOrientation(front_dist, left_dist, right_dist, real_front_dist, real_left_dist, real_right_dist, real_back_dist, needed_to_be);
	cout << "Current: " << curr_o << " Needed: " << needed_to_be << endl;
	changeOrientation(needed_to_be);
}

void Nemala::lastFineTune() {
	int real_front, real_left, real_back, real_right;
	map->getDistances(map->tgt_x, map->tgt_y, curr_o, real_front, real_back, real_right, real_left);
	int front, another;
	front=getMid(readSonar(2),readSonar(2),readSonar(2));
	if (front < real_front-FINAL_DEST_EPSELON) {
		driveBackward(10*abs(real_front-front),-1,-1,real_front);
		stop();
		//front=getMid(readSonar(2),readSonar(2),readSonar(2));
	} else if (front > real_front+FINAL_DEST_EPSELON) {
		driveForward(10*abs(real_front-front),-1,-1,real_front);
		stop();
		//front=getMid(readSonar(2),readSonar(2),readSonar(2));
	}
	int right=getMid(readSonar(4),readSonar(4),readSonar(4));
	int left=getMid(readSonar(0),readSonar(0),readSonar(0));
	another = min(left, right);
	if (left == another) {
		if (left < real_left-FINAL_DEST_EPSELON) {
			turnRight(0.25);
			driveForward(abs(left-real_left));
		} else if (left > real_left+FINAL_DEST_EPSELON) {
			turnLeft(0.25);
			driveForward(abs(left-real_left),-1,-1,real_left);
		}
	} else {
		if (right < real_right-FINAL_DEST_EPSELON) {
			turnLeft(0.25);
			driveForward(abs(right-real_right));
		} else if (left > real_left+FINAL_DEST_EPSELON) {
			turnRight(0.25);
			driveForward(abs(right-real_right),-1,-1,real_right);
		}
	}
}

void Nemala::driveForwardCommand(Speed speed)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x44, 0x01, speed /*replace with DEFAULT_SPEED*/, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::calibrate()
{
	int err_count; // after CALIB_ERRTIMES it will give a push to the other side
	double glob_calib_fix;
	double glob_r_enc, glob_l_enc;
	Distance howlong=500;
	int avg_glob_calib=0;
	int iterations=0;
	glob_calib_avg=0;
	glob_r_enc=0;
	glob_l_enc=0;
	err_count=0;
	glob_calib_fix=(getDriftSpeed()*CALIB_DIV);
	zeroEncoders();
	while(1) {
		short left, right;
		iterations++;
		driveForwardCommand();
	    left = getLeftEncoder();
	    right = getRightEncoder();
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
				setDriftSpeed(CALIB_DRIFTLIMIT);
				avg_glob_calib+=CALIB_DRIFTLIMIT;
			} else {
				setDriftSpeed((Speed)(left-right)/CALIB_DIV);
				avg_glob_calib+=(left-right)/CALIB_DIV;
			}
			setDriftDirection(RIGHT);
	    } else if (right > left+CALIB_TOL) {
			err_count-=1;
			if (err_count < -CALIB_ERRTIMES) {
				left += glob_calib_fix;
				err_count=0;
			}

			if ((right-left)/CALIB_DIV > CALIB_DRIFTLIMIT) {
				setDriftSpeed(CALIB_DRIFTLIMIT);
				avg_glob_calib-=CALIB_DRIFTLIMIT;
			} else {
				setDriftSpeed((right-left)/CALIB_DIV);
				avg_glob_calib-=(right-left)/CALIB_DIV;
			}
			setDriftDirection(LEFT);
		} else {
			setDriftSpeed(0);
			setDriftDirection(LEFT);
		}
		//cout << "Left: " << glob_r_enc << " Right: " << glob_l_enc << " Diff: " << left-right << " and: " << glob_l_enc-glob_r_enc << endl;
		if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
			stop();
			if (avg_glob_calib > 0) {
				glob_calib_avg=avg_glob_calib/iterations;
				glob_calib_dir=RIGHT;
			} else {
				glob_calib_avg=-avg_glob_calib/iterations;
				glob_calib_dir=LEFT;
			}
			return;
		}
	}
}
void Nemala::driveForward(Distance howlong, Distance right_dist, Distance left_dist, Distance front_dist)
{
	cout << "  Driving forward " << howlong << " mm. " << "distances: right/left/front = " << right_dist << " " << left_dist << " " << front_dist << endl;
#ifndef DEBUG
	int err_count; // after CALIB_ERRTIMES it will give a push to the other side
	int glob_calib_fix;
	int glob_r_enc, glob_l_enc;
	int actual_left_dist,actual_front_dist,actual_right_dist;
	if (front_dist > -1) {
		front_dist+=FINAL_DEST_EPSELON;
	}
	glob_r_enc=0;
	glob_l_enc=0;
	err_count=0;
	actual_left_dist=left_dist;
	actual_front_dist=front_dist;
	actual_right_dist=right_dist;
	short global_left_right_by_sonars=0;
	short global_left_by_sonars=0;
	if (left_dist > -1) {
		readSonar(0);
		actual_left_dist=getMid(readSonar(0),readSonar(0),readSonar(0));
	}
	if (right_dist > -1) {
		readSonar(4);
		actual_right_dist=getMid(readSonar(4),readSonar(4),readSonar(4));
	}
	glob_calib_fix=glob_calib_avg*CALIB_DIV;
	zeroEncoders();
	setDriftSpeed(glob_calib_avg);
	//setDriftSpeed(0);
	setDriftDirection(glob_calib_dir?RIGHT:LEFT);
	int last_sonar_front=0;
	int supposed_to_be=howlong;
	if (front_dist > -1) {
		howlong=getMid(readSonar(2), readSonar(2), readSonar(2))-front_dist;
	}
	int sonarcounter=1;
	//int sonarfixes=SONAR_NR_OF_FIXES;
	int sonarfixes=0;
	driveForwardCommand(0x02);
	while(1) {
		short left, right;
		driveForwardCommand();
		left = getLeftEncoder()+global_left_by_sonars+global_left_right_by_sonars;
		right = getRightEncoder()+global_left_right_by_sonars;
		glob_r_enc=right;
		glob_l_enc=left;
		if ((left_dist*right_dist*front_dist!=-1) && ((glob_r_enc + glob_r_enc)/2 > sonarcounter*MM_BETWEEN_SONAR_READS/MM_PER_ENC_TICK)) {
			//setDriftSpeed(glob_calib_avg);
			//setDriftDirection(glob_calib_dir?LEFT:RIGHT);
			//driveForwardCommand(0x10);
			actual_left_dist=left_dist;
			actual_front_dist=front_dist;
			actual_right_dist=right_dist;
			if (left_dist > -1) {
				//actual_left_dist=getMid(readSonar(0),readSonar(0),readSonar(0));
				actual_left_dist=readSonar(0);
				if (abs(left_dist-actual_left_dist) <= SonarDFEpselon) {
					actual_left_dist=left_dist;
				}
			}
			if (right_dist > -1) {
				//actual_right_dist=getMid(readSonar(4),readSonar(4),readSonar(4));
				actual_right_dist=readSonar(4);
				if (abs(right_dist-actual_right_dist) <= SonarDFEpselon) {
					actual_right_dist=right_dist;
				}
			}
			if (front_dist > -1) {
				//stop();
				//actual_front_dist=getMid(readSonar(2),readSonar(2),readSonar(2));
				actual_front_dist=readSonar(2);
				if (abs(front_dist-actual_front_dist) <= SonarDFEpselon) {
					actual_front_dist=front_dist;
				}
				howlong=MM_PER_ENC_TICK*(glob_r_enc + glob_r_enc)/2+10*actual_front_dist-10*front_dist;
				last_sonar_front=actual_front_dist;
				cout << "howlong: " << howlong << " Sonar: " << actual_front_dist << endl;
				//driveForwardCommand();
			}
			sonarcounter++;
			cout << "Left Err: " << actual_left_dist << " Right Err: " << actual_right_dist << endl;
			sonarfixes=0;
		}
		//left+=glob_calib_fix;
		glob_calib_fix=left-right;
		if (sonarfixes < SONAR_NR_OF_FIXES) {
			//if (!sonarfixes) {
				global_left_by_sonars=right-left;
				//global_left_right_by_sonars+=(left+right)/2;
				//zeroEncoders();
			//}
			left+=0.8*(actual_left_dist-left_dist)/SonarDFEpselon;
			right+=0.8*(actual_right_dist-right_dist)/SonarDFEpselon;
			cout << "Sonar fixing total: " << (actual_right_dist-right_dist)/SonarDFEpselon << endl;
			sonarfixes++;
		}
		//nemala.zeroEncoders();
		if (left > right+CALIB_TOL) {
			err_count+=1;
			if (err_count > CALIB_ERRTIMES) {
				left -= glob_calib_fix;
				err_count=0;
			}
			if ((left-right)/CALIB_DIV > CALIB_DRIFTLIMIT) {
				setDriftSpeed(CALIB_DRIFTLIMIT);
				//setDriftSpeed(glob_calib_avg);
			} else {
				setDriftSpeed((left-right)/CALIB_DIV);
			}
			setDriftDirection(RIGHT);
		} else if (right > left+CALIB_TOL) {
			err_count-=1;
			if (err_count < -CALIB_ERRTIMES) {
				left += glob_calib_fix;
				err_count=0;
			}

			if ((right-left)/CALIB_DIV > CALIB_DRIFTLIMIT) {
				setDriftSpeed(CALIB_DRIFTLIMIT);
				//setDriftSpeed(glob_calib_avg);
			} else {
				setDriftSpeed((right-left)/CALIB_DIV);
			}
			setDriftDirection(LEFT);
		} else {
			setDriftSpeed(0);
			setDriftDirection(LEFT);
		}
		//cout << "Left: " << glob_r_enc << " Right: " << glob_l_enc << " Diff: " << left-right << " and: " << glob_l_enc-glob_r_enc << endl;
		if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
			if (front_dist > -1) {
				//stop();
				actual_front_dist=getMid(readSonar(2),readSonar(2),readSonar(2));
				howlong=MM_PER_ENC_TICK*(glob_r_enc + glob_r_enc)/2+10*actual_front_dist-10*front_dist;
				last_sonar_front=actual_front_dist;
				cout << "howlong: " << howlong << " Sonar: " << actual_front_dist << endl;
				//driveForwardCommand();
			}
			if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
				stop();
				setDriftSpeed(glob_calib_avg);
				setDriftDirection(glob_calib_dir?RIGHT:LEFT);
				return;
			}
		}
	}
#endif
}
void Nemala::driveForward2(Distance howlong, Distance right_dist, Distance left_dist, Distance front_dist)
{
	cout << "  Driving forward " << howlong << " mm" << endl;
#ifndef DEBUG
	int err_count; // after CALIB_ERRTIMES it will give a push to the other side
	int glob_calib_fix;
	int glob_r_enc, glob_l_enc;
	int actual_left_dist,actual_front_dist,actual_right_dist;
	glob_r_enc=0;
	glob_l_enc=0;
	err_count=0;
	setDriftSpeed(glob_calib_avg);
	setDriftDirection(glob_calib_dir?RIGHT:LEFT);
	zeroEncoders();
	driveForwardCommand();
	readSonar(0);readSonar(0);readSonar(0);
	readSonar(4);readSonar(4);readSonar(4);
	int sonarcounter=1;
	actual_left_dist=left_dist;
	actual_front_dist=front_dist;
	actual_right_dist=right_dist;
	while(1) {
		short left, right;
		left = getLeftEncoder();
		right = getRightEncoder();
		glob_r_enc=right;
		glob_l_enc=left;
		if ((glob_r_enc + glob_r_enc)/2 > sonarcounter*250/MM_PER_ENC_TICK) {
			actual_left_dist=left_dist;
			actual_front_dist=front_dist;
			actual_right_dist=right_dist;
			if (left_dist > -1) {
				actual_left_dist=getMid(readSonar(0),readSonar(0),readSonar(0));
				if (abs(left_dist-actual_left_dist) <= SonarDFEpselon) {
					actual_left_dist=left_dist;
				}
			}
			if (right_dist > -1) {
				actual_right_dist=getMid(readSonar(4),readSonar(4),readSonar(4));
				if (abs(right_dist-actual_right_dist) <= SonarDFEpselon) {
					actual_right_dist=right_dist;
				}
			}
			if (front_dist > -1) {
				actual_front_dist=readSonar(2);
				if (abs(front_dist-actual_front_dist) <= SonarDFEpselon) {
					actual_front_dist=front_dist;
				}
			}
			sonarcounter++;
			cout << "Left Err: " << actual_left_dist << " Right Err: " << actual_right_dist << endl;
		}
		//left+=glob_calib_fix;
		//glob_calib_fix=left-right;
		//left+=2*(actual_left_dist-left_dist)/SonarDFEpselon;
		//right+=2*(actual_right_dist-right_dist)/SonarDFEpselon;
		//nemala.zeroEncoders();
		//if (left > right+CALIB_TOL) {
			//err_count+=1;
			//if (err_count > CALIB_ERRTIMES) {
				//left -= glob_calib_fix;
				//err_count=0;
			//}
			//if ((left-right)/CALIB_DIV > CALIB_DRIFTLIMIT) {
				//setDriftSpeed(CALIB_DRIFTLIMIT);
			//} else {
				//setDriftSpeed((left-right)/CALIB_DIV);
			//}
			//setDriftDirection(RIGHT);
		//} else if (right > left+CALIB_TOL) {
			//err_count-=1;
			//if (err_count < -CALIB_ERRTIMES) {
				//left += glob_calib_fix;
				//err_count=0;
			//}

			//if ((right-left)/CALIB_DIV > CALIB_DRIFTLIMIT) {
				//setDriftSpeed(CALIB_DRIFTLIMIT);
			//} else {
				//setDriftSpeed((right-left)/CALIB_DIV);
			//}
			//setDriftDirection(LEFT);
		//} else {
			//setDriftSpeed(0);
			//setDriftDirection(LEFT);
		//}
		//cout << "Left: " << glob_r_enc << " Right: " << glob_l_enc << " Diff: " << left-right << " and: " << glob_l_enc-glob_r_enc << endl;
		if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
			stop();
			return;
		}
	}
#endif
}

// 0 - left
// 4 - right
Distance Nemala::readSonar(int sonarNr)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x52, sonarNr, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	Distance dist = _getdistance()/SonarForCM;
	return dist;
}

void Nemala::driveBackwardCommand(Speed speed)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x44, 0x00, speed /*replace with DEFAULT_SPEED*/, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}

void Nemala::driveBackward(Distance howlong, Distance right_dist, Distance left_dist, Distance front_dist)
{
	cout << "  Driving backward " << howlong << " mm. " << "distances: right/left/front = " << right_dist << " " << left_dist << " " << front_dist << endl;
#ifndef DEBUG
	int err_count; // after CALIB_ERRTIMES it will give a push to the other side
	int glob_calib_fix;
	int glob_r_enc, glob_l_enc;
	int actual_left_dist,actual_front_dist,actual_right_dist;
	if (front_dist > -1) {
		front_dist+=FINAL_DEST_EPSELON;
	}
	glob_r_enc=0;
	glob_l_enc=0;
	err_count=0;
	actual_left_dist=left_dist;
	actual_front_dist=front_dist;
	actual_right_dist=right_dist;
	short global_left_right_by_sonars=0;
	short global_left_by_sonars=0;
	if (left_dist > -1) {
		readSonar(0);
		actual_left_dist=getMid(readSonar(0),readSonar(0),readSonar(0));
	}
	if (right_dist > -1) {
		readSonar(4);
		actual_right_dist=getMid(readSonar(4),readSonar(4),readSonar(4));
	}
	glob_calib_fix=glob_calib_avg*CALIB_DIV;
	zeroEncoders();
	setDriftSpeed(glob_calib_avg);
	//setDriftSpeed(0);
	setDriftDirection(glob_calib_dir?RIGHT:LEFT);
	int last_sonar_front=0;
	int supposed_to_be=howlong;
	if (front_dist > -1) {
		howlong=getMid(readSonar(2), readSonar(2), readSonar(2))-front_dist;
	}
	int sonarcounter=1;
	//int sonarfixes=SONAR_NR_OF_FIXES;
	int sonarfixes=0;
	driveBackwardCommand(0x02);
	while(1) {
		short left, right;
		driveBackwardCommand();
		left = INT_MAX-(getLeftEncoder()+global_left_by_sonars+global_left_right_by_sonars);
		right = INT_MAX-(getRightEncoder()+global_left_right_by_sonars);
		glob_r_enc=right;
		glob_l_enc=left;
		if ((left_dist*right_dist*front_dist!=-1) && ((glob_r_enc + glob_r_enc)/2 > sonarcounter*MM_BETWEEN_SONAR_READS/MM_PER_ENC_TICK)) {
			//setDriftSpeed(glob_calib_avg);
			//setDriftDirection(glob_calib_dir?LEFT:RIGHT);
			//driveForwardCommand(0x10);
			actual_left_dist=left_dist;
			actual_front_dist=front_dist;
			actual_right_dist=right_dist;
			if (left_dist > -1) {
				//actual_left_dist=getMid(readSonar(0),readSonar(0),readSonar(0));
				actual_left_dist=readSonar(0);
				if (abs(left_dist-actual_left_dist) <= SonarDFEpselon) {
					actual_left_dist=left_dist;
				}
			}
			if (right_dist > -1) {
				//actual_right_dist=getMid(readSonar(4),readSonar(4),readSonar(4));
				actual_right_dist=readSonar(4);
				if (abs(right_dist-actual_right_dist) <= SonarDFEpselon) {
					actual_right_dist=right_dist;
				}
			}
			if (front_dist > -1) {
				//stop();
				//actual_front_dist=getMid(readSonar(2),readSonar(2),readSonar(2));
				actual_front_dist=readSonar(2);
				if (abs(front_dist-actual_front_dist) <= SonarDFEpselon) {
					actual_front_dist=front_dist;
				}
				howlong=MM_PER_ENC_TICK*(glob_r_enc + glob_r_enc)/2+10*actual_front_dist-10*front_dist;
				last_sonar_front=actual_front_dist;
				cout << "howlong: " << howlong << " Sonar: " << actual_front_dist << endl;
				//driveForwardCommand();
			}
			sonarcounter++;
			cout << "Left Err: " << actual_left_dist << " Right Err: " << actual_right_dist << endl;
			sonarfixes=0;
		}
		//left+=glob_calib_fix;
		glob_calib_fix=left-right;
		if (sonarfixes < SONAR_NR_OF_FIXES) {
			//if (!sonarfixes) {
				global_left_by_sonars=right-left;
				//global_left_right_by_sonars+=(left+right)/2;
				//zeroEncoders();
			//}
			left+=0.8*(actual_left_dist-left_dist)/SonarDFEpselon;
			right+=0.8*(actual_right_dist-right_dist)/SonarDFEpselon;
			cout << "Sonar fixing total: " << (actual_right_dist-right_dist)/SonarDFEpselon << endl;
			sonarfixes++;
		}
		//nemala.zeroEncoders();
		if (left > right+CALIB_TOL) {
			err_count+=1;
			if (err_count > CALIB_ERRTIMES) {
				left -= glob_calib_fix;
				err_count=0;
			}
			if ((left-right)/CALIB_DIV > CALIB_DRIFTLIMIT) {
				setDriftSpeed(CALIB_DRIFTLIMIT);
				//setDriftSpeed(glob_calib_avg);
			} else {
				setDriftSpeed((left-right)/CALIB_DIV);
			}
			setDriftDirection(RIGHT);
		} else if (right > left+CALIB_TOL) {
			err_count-=1;
			if (err_count < -CALIB_ERRTIMES) {
				left += glob_calib_fix;
				err_count=0;
			}

			if ((right-left)/CALIB_DIV > CALIB_DRIFTLIMIT) {
				setDriftSpeed(CALIB_DRIFTLIMIT);
				//setDriftSpeed(glob_calib_avg);
			} else {
				setDriftSpeed((right-left)/CALIB_DIV);
			}
			setDriftDirection(LEFT);
		} else {
			setDriftSpeed(0);
			setDriftDirection(LEFT);
		}
		//cout << "Left: " << glob_r_enc << " Right: " << glob_l_enc << " Diff: " << left-right << " and: " << glob_l_enc-glob_r_enc << endl;
		if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
			if (front_dist > -1) {
				//stop();
				actual_front_dist=getMid(readSonar(2),readSonar(2),readSonar(2));
				howlong=MM_PER_ENC_TICK*(glob_r_enc + glob_r_enc)/2+10*actual_front_dist-10*front_dist;
				last_sonar_front=actual_front_dist;
				cout << "howlong: " << howlong << " Sonar: " << actual_front_dist << endl;
				//driveForwardCommand();
			}
			if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
				stop();
				setDriftSpeed(glob_calib_avg);
				setDriftDirection(glob_calib_dir?RIGHT:LEFT);
				return;
			}
		}
	}
#endif
}

void Nemala::turnLeftCommand(Speed speed)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x54, 0x00, speed, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::turnRightCommand(Speed speed)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x54, 0x01, speed, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::turnRight(float turn_amount_angle)
{
	cout << "  Turning right   " << turn_amount_angle << " circle" << endl;
#ifndef DEBUG
	short left, right;
	int glob_r_enc, glob_l_enc;
	int turn_amount;
	//int turn_speed;
	//turn_speed = 0x00;
	turn_amount=turn_amount_angle*TICKS_PER_360_DEG;
	right=left=0;
	zeroEncoders();
	glob_r_enc=0;
	glob_l_enc=0;
	//nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
	while (1) {
		left = getLeftEncoder()+glob_l_enc;
		right = getRightEncoder()+glob_r_enc;
		glob_r_enc=right;
		glob_l_enc=left;
		if (abs(left-right) < turn_amount-TURN_TOLERANCE) {
			setDriftSpeed(CALIB_DRIFTLIMIT/2);
			setDriftDirection(RIGHT);
			//nemala.setDriftSpeed(0);
			zeroEncoders();
			turnRightCommand(0x10);
			stop();
		} else if (abs(left-right) > turn_amount+TURN_TOLERANCE) {
			for (int i=0; i<abs(left-right)-(turn_amount+TURN_TOLERANCE); i++) {
				setDriftSpeed(CALIB_DRIFTLIMIT/2);
				setDriftDirection(RIGHT);
				zeroEncoders();
				turnLeftCommand(0x10);
				stop();
				left = glob_l_enc-getLeftEncoder();
				right = glob_r_enc-getRightEncoder();
				glob_r_enc=right;
				glob_l_enc=left;
				cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
			}
			setDriftSpeed(0);
			stop();
			return;
		} else {
			setDriftSpeed(0);
			stop();
			cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
			return;
		}
		cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
	}
	//cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
	setDriftSpeed(0);
	stop();
	return;
#endif
}
void Nemala::turnLeft(float turn_amount_angle)
{
	cout << "  Turning left    " << turn_amount_angle << " circle" << endl;
#ifndef DEBUG
	short left, right;
	int drift;
	int glob_r_enc, glob_l_enc;
	int turn_amount;
	//int turn_speed;
	//turn_speed = 0x00;
	turn_amount=(int)(turn_amount_angle*TICKS_PER_360_DEG);
	right=left=0;
	zeroEncoders();
	glob_r_enc=0;
	glob_l_enc=0;
	//nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
	while (1) {
		left = getLeftEncoder()+glob_l_enc;
		right = getRightEncoder()+glob_r_enc;
		glob_r_enc=right;
		glob_l_enc=left;
		if (abs(left-right) < turn_amount-TURN_TOLERANCE) {
			setDriftSpeed(CALIB_DRIFTLIMIT/2);
			setDriftDirection(RIGHT);
			//nemala.setDriftSpeed(0);
			zeroEncoders();
			turnLeftCommand(0x10);
			stop();
		} else if (abs(left-right) > turn_amount+TURN_TOLERANCE) {
			for (int i=0; i<abs(left-right)-(turn_amount+TURN_TOLERANCE); i++) {
				setDriftSpeed(CALIB_DRIFTLIMIT/2);
				setDriftDirection(LEFT);
				turnRightCommand(0x10);
				stop();
				left = glob_l_enc-getLeftEncoder();
				right = glob_r_enc-getRightEncoder();
				glob_r_enc=right;
				glob_l_enc=left;
				cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
			}
			setDriftSpeed(0);
			stop();
			return;
		} else {
			setDriftSpeed(0);
			stop();
			cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
			return;
		}
		cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
	}
	//cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
	setDriftSpeed(0);
	stop();
	return;
#endif
}

void Nemala::turnLeft2(float turn_amount_angle)
{
	cout << "  Turning left    " << turn_amount_angle << " circle" << endl;
#ifndef DEBUG
	short left, right;
	int drift;
	int glob_r_enc, glob_l_enc;
	int turn_amount;
	int speed = 0x09;
	//int turn_speed;
	//turn_speed = 0x00;
	turn_amount=(int)(turn_amount_angle*TICKS_PER_360_DEG);
	right=left=0;
	zeroEncoders();
	glob_r_enc=0;
	glob_l_enc=0;
	//setDriftSpeed(glob_calib_avg);
	setDriftSpeed(CALIB_DRIFTLIMIT);
	setDriftDirection(glob_calib_dir?RIGHT:LEFT);
	turnLeftCommand(speed);
	//nemala.setDriftSpeed(CALIB_DRIFTLIMIT/2);
	while (1) {
		left = getLeftEncoder();
		right = getRightEncoder();
		glob_r_enc=right;
		glob_l_enc=left;
		if (abs(left+right) < turn_amount-TURN_TOLERANCE) {
			//nemala.setDriftSpeed(0);
			//turnLeftCommand(speed);
			cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
		} else if (abs(left+right) > turn_amount+TURN_TOLERANCE) {
			speed=0x02;
			while (abs(left+right) > turn_amount+TURN_TOLERANCE) {
				cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
				stop();
				return;
				turnRightCommand(speed);
				left = getLeftEncoder();
				right = getRightEncoder();
			}
			if (abs(left-right) >= turn_amount-TURN_TOLERANCE) {
				stop();
				cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
				return;
			}
			turnLeftCommand(speed);
			cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
		} else {
			setDriftSpeed(0);
			stop();
			return;
		}
	}
	//cout << "Right: " << right << " Left: " << left << " Until: " << turn_amount << endl;
	setDriftSpeed(0);
	stop();
	return;
#endif
}

void Nemala::stop()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x20, 0x00, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
Speed Nemala::getMaxSpeed()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x47, 0x00, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	Param p = _getparameter();
	return p.speed;
}
Speed Nemala::getMinSpeed()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x47, 0x01, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	Param p = _getparameter();
	return p.speed;
}
Speed Nemala::getDriftSpeed()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x47, 0x02, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	Param p = _getparameter();
	return p.speed;
}
Direction Nemala::getDriftDirection()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x47, 0x03, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	Param p = _getparameter();
	return p.direction;
}

short Nemala::getLeftEncoder()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x45, 0x00, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	return _getencoder();
}
short Nemala::getRightEncoder()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x45, 0x01, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	return _getencoder();
}

void Nemala::setMaxSpeed(Speed speed)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x53, 0x00, (char)speed, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::setMinSpeed(Speed speed)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x53, 0x01, (char)speed, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::setDriftSpeed(Speed speed)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x53, 0x02, (char)speed, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::setDriftDirection(Direction direction)
{
	char towrite[BUFF_SIZE];
	if (direction == LEFT) {
		_dataprepare(0x53, 0x03, (char)0, towrite);
	} else {
		_dataprepare(0x53, 0x03, (char)1, towrite);
	}
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::terminate()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x7E, 0x00, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
}

void Nemala::_dataprepare(char a, char b, char c, char towrite[4]) {
	towrite[0]=a;
	towrite[1]=b;
	towrite[2]=c;
	towrite[3]=CHECK_SUM(a,b,c);
}

bool Nemala::_verifybuffer(char buff[4]) {
	return (buff[3] == (CHECK_SUM(buff[0],buff[1],buff[2])));
}

DWORD Nemala::_readfrombuff(char* szBuffer, size_t size, bool blocking) {
	// Read data, until there is nothing left with minimum number of bytes
	DWORD dwBytesRead = 0;
	if (blocking) {
		cs.SetupReadTimeouts(CSerial::EReadTimeoutBlocking);
	} else {
		cs.SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
	}
	// Read data from the COM-port
	LONG    lLastError = cs.Read(szBuffer,size,&dwBytesRead);
	if (lLastError != ERROR_SUCCESS) {
		return -1;
	}

	return dwBytesRead;
}

void Nemala::_waitforv() {
	DWORD szRead=0;
	char szBuffer[5];
	szRead=_readfrombuff(szBuffer, 4);
	if ((_verifybuffer(szBuffer)) && (szBuffer[0]=='v')) {
		//do nothing
	} else {
		throw WAIT_FOR_V_EXCEPTION;
	}
}



Param Nemala::_getparameter(){
	bool release=false;
	DWORD szRead=0;
	char szBuffer[101];
	szRead=_readfrombuff(szBuffer, 4);
	if (_verifybuffer(szBuffer) && (szBuffer[0]=='g')) {
		Param p = {szBuffer[2]};
		return p;
	} 
	else 
	{
		throw GET_PARAMETER_EXCEPTION;
	}
}

short Nemala::_getencoder() {
	DWORD szRead=0;
	char szBuffer[101];
	szRead=_readfrombuff(szBuffer, 4);
	if (_verifybuffer(szBuffer) && (szBuffer[0]=='e')) {
		return (short)((((short)szBuffer[2] << 8) & 0xFF00) + ((short)szBuffer[1] & 0xFF)) & 0xFFFF;
	} else {
		throw GET_ENCODER_EXCEPTION;
	}
}
int Nemala::_getdistance() {
	DWORD szRead=0;
	char szBuffer[101];
	szRead=_readfrombuff(szBuffer, 4);
	if (_verifybuffer(szBuffer) && (szBuffer[0]=='r')) {
		return (int)((((int)szBuffer[2] << 8) & 0xFF00) + ((int)szBuffer[1] & 0xFF)) & 0xFFFF;
	} else {
		throw GET_ENCODER_EXCEPTION;
	}
}
void Nemala::zeroEncoders()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x30, 0x00, 0x00, towrite);
	while(true)
	{
		cs.Write(towrite, 4*sizeof(char));
		try
		{
			_waitforv();
		}
		catch (NemalaException ne)
		{
			continue;
		}
		break;
	}
	_dataprepare(0x30, 0x01, 0x00, towrite);
	while(true)
	{
		cs.Write(towrite, 4*sizeof(char));
		try
		{
			_waitforv();
		}
		catch (NemalaException ne)
		{
			continue;
		}
		break;
	}
}

void Nemala::_connect()
{
	LONG lLastError = ERROR_SUCCESS;
	lLastError = cs.Open(_T(COMPORT));
	if (lLastError == ERROR_SUCCESS) 
	{
		cout << "Port: " << COMPORT << " available, and connected successfully" << endl;
	}
	else
	{
		throw PORT_NOT_AVAILABLE_EXCEPTION;
	}
}

void Nemala::_disconnect()
{
	cs.Close();
}
void Nemala::fineTune()
{	
	/*TODO: implement*/
}

void Nemala::changeOrientation(Orientation o)
{
	if(curr_o == o) return;

	if(curr_o == NORTH && o == WEST ) turnLeft (0.25);
	if(curr_o == NORTH && o == SOUTH) turnLeft (0.5);
	if(curr_o == NORTH && o == EAST ) turnRight(0.25);

	if(curr_o == WEST  && o == SOUTH) turnLeft (0.25);
	if(curr_o == WEST  && o == EAST ) turnLeft (0.5);
	if(curr_o == WEST  && o == NORTH) turnRight(0.25);

	if(curr_o == SOUTH && o == EAST ) turnLeft (0.25);
	if(curr_o == SOUTH && o == NORTH) turnLeft (0.5);
	if(curr_o == SOUTH && o == WEST ) turnRight(0.25);

	if(curr_o == EAST  && o == NORTH) turnLeft (0.25);
	if(curr_o == EAST  && o == WEST ) turnLeft (0.5);
	if(curr_o == EAST  && o == SOUTH) turnRight(0.25);

	curr_o = o;
}

void Nemala::driveXaxis(int xFrom, int xTo, int y, StationType st)
{	
	//cout << "driveXaxis from " << xFrom << "to " << xTo;
	int dist = abs(xFrom-xTo);
	int frontDist, rightDist, leftDist, backDist;

	if(xFrom < xTo) 
		changeOrientation(WEST);
	else
		changeOrientation(EAST);

	map->getDistances(xTo, y, curr_o, frontDist, backDist, rightDist, leftDist);

	if(st == LAST)
	{
		if(rightDist < leftDist)
			driveForward(dist*10,rightDist, -1, frontDist);
		else
			driveForward(dist*10,-1, leftDist, frontDist);
	}
	else if(st == BEFORE_LAST)
	{
		driveForward(dist*10,-1, -1, frontDist);
	}
	else if(st == FIRST)
	{
		if(rightDist < leftDist)
			driveForward(dist*10,rightDist, -1, -1);
		else
			driveForward(dist*10, -1, leftDist, -1);
	}
	else if(st == MIDDLE)
	{
		driveForward(dist*10);
	}
	else
	{
		throw "Bad station type";
	}
}

void Nemala::driveYaxis(int yFrom, int yTo, int x, StationType st)
{	
	//cout << "driveYaxis from " << yFrom << "to " << yTo;
	int dist = abs(yFrom-yTo);
	int frontDist, rightDist, leftDist, backDist;
	map->getDistances(x, yTo, curr_o, frontDist, backDist, rightDist, leftDist);

	if(yFrom < yTo) 
		changeOrientation(NORTH);
	else
		changeOrientation(SOUTH);

	if(st == LAST)
	{
		if(rightDist < leftDist)
			driveForward(dist*10,rightDist, -1, frontDist);
		else
			driveForward(dist*10,-1, leftDist, frontDist);
	}
	else if(st == BEFORE_LAST)
	{
		driveForward(dist*10,-1, -1, frontDist);
	}
	else if(st == FIRST)
	{
		if(rightDist < leftDist)
			driveForward(dist*10,rightDist, -1, -1);
		else
			driveForward(dist*10, -1, leftDist, -1);
	}
	else if(st == MIDDLE)
	{
		driveForward(dist*10);
	}
	else
	{
		throw "Bad station type";
	}
}