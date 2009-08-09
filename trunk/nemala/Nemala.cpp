#include "Nemala.h"

// defines
#define STRICT
#define COMPORT "COM1"
#define CHECK_SUM(x,y,z) ((char)(((char)x+(char)y+(char)z)%256))
#define BUFF_SIZE 5
#define DEFAULT_SPEED 30

Nemala::Nemala()
{
	_connect();
	zeroEncoders();
}

Nemala::~Nemala()
{
	_disconnect();
}

void Nemala::driveForwardCommand()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x44, 0x01, 0x30 /*replace with DEFAULT_SPEED*/, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}

void Nemala::driveForward(Distance howlong)
{
	int err_count; // after CALIB_ERRTIMES it will give a push to the other side
	int glob_calib_fix;
	int glob_r_enc, glob_l_enc;
	glob_r_enc=0;
	glob_l_enc=0;
	err_count=0;
	glob_calib_fix=getDriftSpeed()*CALIB_DIV;
	zeroEncoders();
	while(1) {
		short left, right;
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
			} else {
				setDriftSpeed((right-left)/CALIB_DIV);
			}
			setDriftDirection(LEFT);
		} else {
			setDriftSpeed(0);
			setDriftDirection(LEFT);
		}
		cout << "Left: " << glob_r_enc << " Right: " << glob_l_enc << " Diff: " << left-right << " and: " << glob_l_enc-glob_r_enc << endl;
		if ((glob_r_enc+glob_l_enc)/2.0 >= (howlong)/MM_PER_ENC_TICK) {
			stop();
			return;
		}
	}
}

Distance Nemala::readSonar(int sonarNr)
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x52, sonarNr, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	Distance dist = _getdistance();
	return dist;
}

void Nemala::driveBackward()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x44, 0x00, 0x30 /*replace with DEFAULT_SPEED*/, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
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
	short left, right;
	int drift;
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
		if ((left+right) < turn_amount-TURN_TOLERANCE) {
			setDriftSpeed(CALIB_DRIFTLIMIT/2);
			setDriftDirection(RIGHT);
			//nemala.setDriftSpeed(0);
			zeroEncoders();
			turnRightCommand(0x20);
			stop();
		} else if ((left+right) > turn_amount+TURN_TOLERANCE) {
			while ((left+right) > turn_amount+TURN_TOLERANCE) {
				setDriftSpeed(CALIB_DRIFTLIMIT/2);
				setDriftDirection(RIGHT);
				zeroEncoders();
				turnLeftCommand(0x02);
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
}
void Nemala::turnLeft(float turn_amount_angle)
{
	short left, right;
	int drift;
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
		if ((left+right) < turn_amount-TURN_TOLERANCE) {
			setDriftSpeed(CALIB_DRIFTLIMIT/2);
			setDriftDirection(RIGHT);
			//nemala.setDriftSpeed(0);
			zeroEncoders();
			turnLeftCommand(0x20);
			stop();
		} else if ((left+right) > turn_amount+TURN_TOLERANCE) {
			while ((left+right) > turn_amount+TURN_TOLERANCE) {
				setDriftSpeed(CALIB_DRIFTLIMIT/2);
				setDriftDirection(RIGHT);
				zeroEncoders();
				turnRightCommand(0x02);
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

