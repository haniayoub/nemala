#include <tchar.h>
#include <windows.h>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include <conio.h>
#include "..\CSerial\Serial.h"

using namespace std;

typedef int Speed;
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

class Nemala
{
public:
	Nemala();
	void		driveForward();
	void		driveBackward();
	void		turnLeft();
	void		turnRight();
	void		stop();
	Speed		getMaxSpeed();
	Speed		getMinSpeed();
	Speed		getDriftSpeed();
	Direction	getDriftDirection();
	int			getLeftEncoder();
	int			getRightEncoder();
	void		setMaxSpeed(Speed speed);
	void		setMinSpeed(Speed speed);
	void		setDriftSpeed(Speed speed);
	void		setDriftDirection(Direction direction);
	void		zeroEncoders();
	void		terminate();
	virtual	~Nemala();
private:
	CSerial cs;
	void	_dataprepare(char a, char b, char c, char towrite[4]);
	bool	_verifybuffer(char buff[4]);
	DWORD	_readfrombuff(char* szBuffer, size_t size, bool blocking = true);
	void	_waitforv();
	Param	_getparameter();
	int		_getencoder();
	void	_connect();
	void	_disconnect();
};