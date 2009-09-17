#include <tchar.h>
#include <windows.h>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include <conio.h>
#include "..\CSerial\Serial.h"
#include "..\map\Map.h"


#define DEFAULT_SPEED 0x1C
#define CALIB_DIV 1.1578
#define CALIB_TOL 0
#define CALIB_ERRTIMES 2999
//#define CALIB_DRIFTLIMIT 20
//#define MM_PER_ENC_TICK 1.7640573318632855567805953693495
#define MM_PER_ENC_TICK 1.86206051696680142104618400098
//#define MM_PER_360_DEG 200
//#define TICKS_PER_360_DEG 316
//#define TURN_TOLERANCE 0

using namespace std;

typedef int Speed;
typedef int Distance;
typedef enum {LEFT=0, RIGHT=1} Direction;
typedef union Param
{
	Speed speed;
	Direction direction;
} Param;

typedef enum {	
	GET_PARAMETER_EXCEPTION, 
	WAIT_FOR_V_EXCEPTION, 
	GET_ENCODER_EXCEPTION, 
	PORT_NOT_AVAILABLE_EXCEPTION
} NemalaException;

typedef enum {	
	BLOCKED, 
	FREE
} BUG_STATE;

typedef enum {	
	oLEFT, 
	oRIGHT
} OBS_POS;


int getMid(int x, int y, int z);

class Nemala
{
public:
	Nemala(Map *map, Orientation o);
	Map			*map;
	void		driveForwardCommand(Speed speed=DEFAULT_SPEED);
	void		driveBackwardCommand(Speed speed=DEFAULT_SPEED);
	void		driveForward(Distance howlong=100, Distance right_dist=-1, Distance left_dist=-1, Distance front_dist=-1, Speed s=DEFAULT_SPEED, Distance skipuntillast=INT_MAX);
	BUG_STATE	driveForwardBug(Distance howlong=100, Distance right_dist=-1, Distance left_dist=-1, Distance front_dist=-1, Speed s=DEFAULT_SPEED, bool secondbypass=false, Distance skipuntillast=INT_MAX);
	void		driveForward2(Distance howlong=100, Distance right_dist=-1, Distance left_dist=-1, Distance front_dist=-1);
	void		driveBackward(Distance howlong=100, Distance right_dist=-1, Distance left_dist=-1, Distance front_dist=-1);
	void		turnLeftCommand(Speed speed=DEFAULT_SPEED);
	void		turnLeft(float turn_amount_angle=0.25);
	void		turnLeft2(float turn_amount_angle=0.25);
	void		turnRightCommand(Speed speed=DEFAULT_SPEED);
	void		turnRight(float turn_amount_angle=0.25);
	void		stop();
	void		stopFix();
	void		firstFineTune();
	void		lastFineTune();
	void		calibrate();
	Speed		getMaxSpeed();
	Speed		getMinSpeed();
	Speed		getDriftSpeed();
	Direction	getDriftDirection();
	short		getLeftEncoder();
	short		getRightEncoder();
	Distance	readSonar(int sonarNr);
	void		setMaxSpeed(Speed speed);
	void		setMinSpeed(Speed speed);
	void		setDriftSpeed(Speed speed);
	void		setDriftDirection(Direction direction);
	void		zeroEncoders();
	void		terminate();
	void		fineTune();
	void		driveXaxis(int xFrom, int xTo, int y, StationType st, Distance skipuntillast=INT_MAX);
	void		driveYaxis(int yFrom, int yTo, int x, StationType st, Distance skipuntillast=INT_MAX);
	BUG_STATE	driveXaxis_BUG(int xFrom, int xTo, int y, StationType st);
	BUG_STATE	driveYaxis_BUG(int yFrom, int yTo, int x, StationType st);
	void		changeOrientation(Orientation o);
	void		changeOrientationFineTune(Orientation o);
	int			glob_calib_avg;
	int			glob_calib_dir;
	Orientation curr_o;
	virtual		~Nemala();
	OBS_POS		leftByPass();
	OBS_POS		rightByPass();
	BUG_STATE		secondByPass(OBS_POS p);
	OBS_POS		bypass();
	void		bypass2ndChance(OBS_POS op);
	int curr_x, curr_y;

private:
	CSerial cs;
	void	_dataprepare(char a, char b, char c, char towrite[4]);
	bool	_verifybuffer(char buff[4]);
	DWORD	_readfrombuff(char* szBuffer, size_t size, bool blocking = true);
	void	_waitforv();
	void	stopTurn();
	Param	_getparameter();
	short	_getencoder();
	Distance _getdistance();
	void	_connect();
	void	_disconnect();
	Orientation figureMyOrientation(int my_front, int my_left, int my_right, int needed_front, int needed_left, int needed_right, int needed_back, Orientation needed_o);
	void turnRight90();
	void turnLeft90();
	void updatePosition(int encoders);
	void updatePositionUsingSonars();
};