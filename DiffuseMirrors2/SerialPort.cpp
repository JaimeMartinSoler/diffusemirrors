#include "SerialPort.h"


void SerialPort::SetParams(int BaudRate, int StopBits) {

		// Step 2: Close file if open
		if (hSerial) CloseHandle(hSerial); 

		// Step 1: Open file
		hSerial = CreateFile(fname,
			/*GENERIC_READ | */GENERIC_WRITE,
			0,
			0,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			0);
		if(hSerial==INVALID_HANDLE_VALUE){
			if(GetLastError()==ERROR_FILE_NOT_FOUND){
				std::cout << "serial port does not exist." <<std::endl;
			}
			std::cout << "some other error occurred."<<std::endl;
		}



		// Step 2: Get port parameters and set custom params
		DCB dcbSerialParams = {0};
		dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
		if (!GetCommState(hSerial, &dcbSerialParams)) {
			std::cout << "error getting serial port state"<<std::endl;
		}
		dcbSerialParams.BaudRate=BaudRate;
		dcbSerialParams.ByteSize=8;
		dcbSerialParams.StopBits=StopBits;
		dcbSerialParams.Parity=NOPARITY;
		if(!SetCommState(hSerial, &dcbSerialParams)){
			std::cout << "error setting serial port state"<<std::endl;
		}

		// Step 3: Disable timeout

		// Tell Windows not to wait for data! Disable timeout
		COMMTIMEOUTS timeouts={0};
		timeouts.ReadIntervalTimeout=50;
		timeouts.ReadTotalTimeoutConstant=50;
		timeouts.ReadTotalTimeoutMultiplier=10;

		timeouts.WriteTotalTimeoutConstant=50;
		timeouts.WriteTotalTimeoutMultiplier=10;
		if(!SetCommTimeouts(hSerial, &timeouts)){
			std::cout << "error occureed setting timeout."<<std::endl;
		}

}
	SerialPort::SerialPort(const char *Name, int BaudRate, int StopBits){

		// Step 1: Open file
		wsprintf(fname, L"%S", Name);
		hSerial = CreateFile(fname,
			/*GENERIC_READ | */GENERIC_WRITE,
			0,
			0,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			0);
		if(hSerial==INVALID_HANDLE_VALUE){
			if(GetLastError()==ERROR_FILE_NOT_FOUND){
				std::cout << "serial port does not exist." <<std::endl;
			}
			std::cout << "some other error occurred."<<std::endl;
		}



		// Step 2: Get port parameters and set custom params
		DCB dcbSerialParams = {0};
		dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
		if (!GetCommState(hSerial, &dcbSerialParams)) {
			std::cout << "error getting serial port state"<<std::endl;
		}
		dcbSerialParams.BaudRate=BaudRate;
		dcbSerialParams.ByteSize=8;
		dcbSerialParams.StopBits=StopBits;
		dcbSerialParams.Parity=NOPARITY;
		if(!SetCommState(hSerial, &dcbSerialParams)){
			std::cout << "error setting serial port state"<<std::endl;
		}

		// Step 3: Disable timeout

		// Tell Windows not to wait for data! Disable timeout
		COMMTIMEOUTS timeouts={0};
		timeouts.ReadIntervalTimeout=50;
		timeouts.ReadTotalTimeoutConstant=50;
		timeouts.ReadTotalTimeoutMultiplier=10;

		timeouts.WriteTotalTimeoutConstant=50;
		timeouts.WriteTotalTimeoutMultiplier=10;
		if(!SetCommTimeouts(hSerial, &timeouts)){
			std::cout << "error occurred setting timeout."<<std::endl;
		}

	};

void SerialPort::WriteChar(char ch) {
	DWORD dwBytesWritten = 0;

	if(!WriteFile(hSerial, &ch, 1, &dwBytesWritten, NULL)){
		std::cout << "error occurred writing data."<<std::endl;
	}

}


void SerialPort::WriteString(const char *str) {
	DWORD dwBytesWritten = 0;

	if(!WriteFile(hSerial, str, strlen(str), &dwBytesWritten, NULL)){
		std::cout << "error occurred writing data."<<std::endl;
	}

}