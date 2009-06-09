#define STRICT
#include <tchar.h>
#include <windows.h>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include "Serial.h"

// Usings
using namespace std;

// Settings
#define COMPORT "COM5"

// MACROs
#define CHECK_SUM(x) ((x)%256)

enum { EOF_Char = 27 };

void _dataprepare(char a, char b, char c, char towrite[4]);
bool _verifybuffer(char buff[4]);
DWORD _readfrombuff(CSerial* serial, char* szBuffer, size_t size, size_t min);
void _switchtoreleasemode(CSerial* serial);
bool _waitforv(CSerial* serial);
bool _waitforinit(CSerial* serial);
bool _getparameter(CSerial* serial, int* param);

int main(int argc, char *argv[])
{
	CSerial serial;
	int param;
	char towrite[5];
	LONG    lLastError = ERROR_SUCCESS;
	lLastError = serial.Open(_T(COMPORT));
	int whattodo;
	if (lLastError == ERROR_SUCCESS) {
		cout << COMPORT << " Port available, and connected successfully" << endl;
		serial.SetMask(CSerial::EEventBreak |
			CSerial::EEventCTS   |
			CSerial::EEventDSR   |
			CSerial::EEventError |
			CSerial::EEventRing  |
			CSerial::EEventRLSD  |
			CSerial::EEventRecv);
		bool fContinue = true;

		//Switch to release mode
		_waitforinit(&serial);
		while (fContinue) {
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
			cout << "-1. TERMINATE!" << endl;
			cin >> whattodo;
			status = false;
			gstatus = false;
			switch (whattodo) {
				   case -1: _dataprepare(0x7E, 0x00, 0x00, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   _waitforinit(&serial);
					   break;
				   case 1: _dataprepare(0x44, 0x01, 0x30, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   status = _waitforv(&serial);
					   break;
				   case 2: _dataprepare(0x44, 0x00, 0x30, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   status = _waitforv(&serial);
					   break;
				   case 30: _dataprepare(0x54, 0x00, 0x20, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   status = _waitforv(&serial);
					   break;
				   case 31: _dataprepare(0x54, 0x01, 0x20, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   status = _waitforv(&serial);
					   break;
				   case 4: _dataprepare(0x20, 0x00, 0x00, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   status = _waitforv(&serial);
					   break;
				   case 50: _dataprepare(0x47, 0x00, 0x00, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   gstatus = _getparameter(&serial, &param);
					   break;
				   case 51: _dataprepare(0x47, 0x01, 0x00, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   gstatus = _getparameter(&serial, &param);
					   break;
				   case 52: _dataprepare(0x47, 0x02, 0x00, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   gstatus = _getparameter(&serial, &param);
					   break;
				   case 53: _dataprepare(0x47, 0x03, 0x00, towrite);
					   serial.Write(towrite, 4*sizeof(char));
					   gstatus = _getparameter(&serial, &param);
					   break;
			}
			if (status) {
				cout << "Operation Completed sccessfully!" << endl;
			}
			if (gstatus) {
				cout << "The Parameter you requested is: " << param << endl;
			}
		}
		serial.Close();
	} else {
		cout << "Port not Available" << endl;
	}
	system("PAUSE");
	return EXIT_SUCCESS;
}
void _dataprepare(char a, char b, char c, char towrite[4]) {
	towrite[0]=a;
	towrite[1]=b;
	towrite[2]=c;
	towrite[3]=CHECK_SUM(a+b+c);
}

bool _verifybuffer(char buff[4]) {
	return ((unsigned int)buff[3] == ((unsigned int)CHECK_SUM(buff[0]+buff[1]+buff[2])));
}

DWORD _readfrombuff(CSerial* serial, char* szBuffer, size_t size, size_t min=0) {
	// Read data, until there is nothing left with minimum number of bytes
	DWORD dwBytesRead = 0;
	DWORD dwLocalBytesRead = 0;
	int i=0;
	// Read data from the COM-port
	do {
		LONG    lLastError = serial->Read(&szBuffer[dwBytesRead],size-dwBytesRead,&dwLocalBytesRead);
		dwBytesRead+=dwLocalBytesRead;
		if (dwLocalBytesRead == 0) {
			i++;
		} else {
			i=0;
		}
		if (lLastError != ERROR_SUCCESS) {
			return -1;
		}
	} while ((dwBytesRead<min));
	/*
	if (dwBytesRead > 0)
	{
	// Finalize the data, so it is a valid string
	//szBuffer[dwBytesRead] = '\0';
	//bool validbuf = _verifybuffer(szBuffer);
	// Display the data
	//if (validbuf) {
	//szBuffer[4]='\0';
	//printf("%s\n", szBuffer);
	//}

	}
	*/
	return dwBytesRead;
}
void _switchtoreleasemode(CSerial* serial) {
	bool release = false;
	CSerial::EEvent eEvent;
	char towrite[4];
	char tempbuff[101];
	do {
		_readfrombuff(serial,tempbuff, 100); // clear the com port buffer
		DWORD szWrite=0;
		_dataprepare(0x4D, 0x00, 0x00, towrite);
		do {
			DWORD szLocalWrite;
			serial->Write(&towrite[szWrite], 4-szWrite, &szLocalWrite);
			szWrite+=szLocalWrite;
		} while (szWrite < 4);
		serial->WaitEvent();
		eEvent = serial->GetEventType();
		//cout << eEvent << endl;
		// Handle data receive event
		if (eEvent & CSerial::EEventRecv) {
			DWORD szRead=0;
			char szBuffer[5];
			szRead=_readfrombuff(serial,szBuffer, 4, 4);
			if (_verifybuffer(szBuffer)) {
				if (szBuffer[0] == 'v') {
					cout << "Robot now in Release mode!" << endl;
					release = true;
				}
			}
		}
	} while (!release);
}

bool _waitforv(CSerial* serial) {
	char tempbuf[100];
	CSerial::EEvent eEvent;
	serial->WaitEvent();
	eEvent = serial->GetEventType();
	//cout << eEvent << endl;
	// Handle data receive event
	if (eEvent & CSerial::EEventRecv)
	{
		DWORD szRead=0;
		char szBuffer[5];
		do {
			szRead=_readfrombuff(serial,szBuffer, 4, 4);
			if (_verifybuffer(szBuffer)) {
				if (szBuffer[0]=='v') {
					while (_readfrombuff(serial,tempbuf, 100)); //empty the buffer after the v
					return true;
				}
			}
		} while (szRead>0);
	}
	return false;
}

bool _waitforinit(CSerial* serial) {
	CSerial::EEvent eEvent;
	bool release=false;
	char towrite[5];

	_dataprepare(0x20, 0x00, 0x00, towrite);
	serial->Write(towrite, 4*sizeof(char));

	serial->WaitEvent();
	eEvent = serial->GetEventType();
	//cout << eEvent << endl;
	// Handle data receive event
	if (eEvent & CSerial::EEventRecv)
	{
		DWORD szRead=0;
		char szBuffer[101];
		do {
			szRead=_readfrombuff(serial,szBuffer, 100);
			if (!release) {
				cout << "Looks like the system initiated, reloading in release mode" << endl;
				_switchtoreleasemode(serial);
				release=true;
			}
		} while (szRead>0);
	}
	return true;
}

bool _getparameter(CSerial* serial, int* param) {
	CSerial::EEvent eEvent;
	bool release=false;

	serial->WaitEvent();
	eEvent = serial->GetEventType();
	//cout << eEvent << endl;
	// Handle data receive event
	if (eEvent & CSerial::EEventRecv)
	{
		DWORD szRead=0;
		char szBuffer[101];
		char tempbuf[101];
		do {
			szRead=_readfrombuff(serial,szBuffer, 4, 4);
			cout << _verifybuffer(szBuffer) << endl;
			if (1+_verifybuffer(szBuffer) && (szBuffer[0]=='g')) {
				*param=(unsigned int)szBuffer[2];
				while (_readfrombuff(serial,tempbuf, 100)); //empty the buffer after the data we got
				return true;
			}
		} while (szRead>0);
	}
	return false;
}
