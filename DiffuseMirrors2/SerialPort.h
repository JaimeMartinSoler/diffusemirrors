#ifndef SERIALPORT_H

#define SERIALPORT_H


#include <windows.h>
#include <iostream>
#include <stdio.h>


class SerialPort {
public:
	HANDLE hSerial;
	WCHAR fname[256];
	SerialPort::SerialPort(const char *Name, int BaudRate = CBR_9600, int StopBits = TWOSTOPBITS);
	~SerialPort() { if (hSerial) CloseHandle(hSerial); 
	};

	
	void SetParams(int BaudRate = CBR_9600, int StopBits = TWOSTOPBITS);
	void WriteString(const char *str);
	void WriteChar(char ch);
	void WriteOp(char ch) {WriteChar(1);WriteChar(ch);};
	void WriteData(char ch) {WriteChar(0);WriteChar(ch);};

};




#endif