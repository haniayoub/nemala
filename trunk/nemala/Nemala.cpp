#include "Nemala.h"

// defines
#define STRICT
#define COMPORT "COM2"
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

void Nemala::driveForward()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x44, 0x01, 0x30 /*replace with DEFAULT_SPEED*/, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
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
void Nemala::turnLeft()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x54, 0x00, 0x20, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
}
void Nemala::turnRight()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x54, 0x01, 0x09, towrite);
	cs.Write(towrite, 4*sizeof(char));
	_waitforv();
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

int Nemala::getLeftEncoder()
{
	char towrite[BUFF_SIZE];
	_dataprepare(0x45, 0x00, 0x00, towrite);
	cs.Write(towrite, 4*sizeof(char));
	return _getencoder();
}
int Nemala::getRightEncoder()
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

int Nemala::_getencoder() {
	DWORD szRead=0;
	char szBuffer[101];
	szRead=_readfrombuff(szBuffer, 4);
	if (_verifybuffer(szBuffer) && (szBuffer[0]=='e')) {
		return (int)((((int)szBuffer[2] << 8) & 0xFF00) + ((int)szBuffer[1] & 0xFF)) & 0xFFFF;
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

