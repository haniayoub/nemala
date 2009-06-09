#define STRICT
#include <tchar.h>
#include <windows.h>
#include <cstdlib>
#include <iostream>
#include <string.h>
#include "..\CSerial\Serial.h"

// Usings
using namespace std;

// Settings
#define COMPORT "COM5"

// MACROs
#define CHECK_SUM(x,y,z) ((char)(((char)x+(char)y+(char)z)%256))

enum { EOF_Char = 27 };

void _dataprepare(char a, char b, char c, char towrite[4]);
bool _verifybuffer(char buff[4]);
DWORD _readfrombuff(CSerial* serial, char* szBuffer, size_t size, bool blocking = true);
bool _waitforv(CSerial* serial);
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
		bool fContinue = true;

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
					   fContinue=false;
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
	towrite[3]=CHECK_SUM(a,b,c);
}

bool _verifybuffer(char buff[4]) {
	return (buff[3] == (CHECK_SUM(buff[0],buff[1],buff[2])));
}

DWORD _readfrombuff(CSerial* serial, char* szBuffer, size_t size, bool blocking) {
	// Read data, until there is nothing left with minimum number of bytes
	DWORD dwBytesRead = 0;
	if (blocking) {
		serial->SetupReadTimeouts(CSerial::EReadTimeoutBlocking);
	} else {
		serial->SetupReadTimeouts(CSerial::EReadTimeoutNonblocking);
	}
	// Read data from the COM-port
	LONG    lLastError = serial->Read(szBuffer,size,&dwBytesRead);
	if (lLastError != ERROR_SUCCESS) {
		return -1;
	}

	return dwBytesRead;
}

bool _waitforv(CSerial* serial) {
	DWORD szRead=0;
	char szBuffer[5];
	szRead=_readfrombuff(serial,szBuffer, 4);
	if ((_verifybuffer(szBuffer)) && (szBuffer[0]=='v')) {
		return true;
	} else {
		return false;
	}
	return false;
}

bool _getparameter(CSerial* serial, int* param) {
	bool release=false;
	DWORD szRead=0;
	char szBuffer[101];
	szRead=_readfrombuff(serial,szBuffer, 4);
	if (_verifybuffer(szBuffer) && (szBuffer[0]=='g')) {
		*param=(unsigned int)szBuffer[2];
		return true;
	} else {
		return false;
	}
	return false;
}
