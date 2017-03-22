/*********************************************************************
 * PiCubes - Native libarry for Pi-Cubes Modules
 *
 * Copyright (c) 2015 Cube-Controls Inc.
 *
 * MIT License
 ********************************************************************/

#include <nan.h>
#include "readui.h"  // NOLINT(build/include)
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

using v8::Function;
using v8::Local;
using v8::Number;
using v8::Value;
using v8::Array;
using Nan::AsyncQueueWorker;
using Nan::AsyncWorker;
using Nan::Callback;
using Nan::HandleScope;
using Nan::New;
using Nan::Null;
using Nan::To;

class readUIWorker : public AsyncWorker {
 public:
  readUIWorker(Callback *callback, int module, int input, int typeui)
    : AsyncWorker(callback), module(module), input(input), typeui(typeui) {}
  ~readUIWorker() {}

  // CRC checking for AsyncWorker
  unsigned int CRC16(unsigned char *buffer, unsigned char messageLength) {
  	unsigned int crc = 0xFFFF;
  	unsigned int crcHigh = 0;
  	unsigned int crcLow = 0;
  	int i, j = 0;

  	for (i = 0;i < messageLength;i++)
  	{
  		crc ^= buffer[i];
  		for (j = 8; j != 0; j--)
  		{
  			if ((crc & 0x0001) != 0)
  			{
  				crc >>= 1;
  				crc ^= 0xA001;
  			}
  			else
  			{
  				crc >>= 1;
  			}
  		}
  	}
  	//bytes are wrong way round so doing a swap here..
  	crcHigh = (crc & 0x00FF) << 8;
  	crcLow = (crc & 0xFF00) >> 8;
  	crcHigh |= crcLow;
  	crc = crcHigh;
  	return crc;
  }

  // Executed inside the worker-thread.
  // It is not safe to access V8, or V8 data structures
  // here, so everything we need for input and output
  // should go on `this`.
  void Execute () {
    unsigned char buf[20];
    int fd;
 	unsigned char inputBytes[16];
 	unsigned int calcCRC = 0;
 	unsigned int inputCRC = 0;

    // Open port for reading and writing
    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
    {
		SetErrorMessage("Failed to open i2c-1 port.");
		return;
    }

	// Set the port options and set the address of the device we wish to speak to
	if (ioctl(fd, I2C_SLAVE, 0x12) < 0)
	{
		SetErrorMessage("Unable to get bus access to talk to Pi-Cubes I/O module.");
		return;
	}

	// calculate address
	buf[0] = 0xAA;

	// Send read register 0xAA
	if ((write(fd, buf, 1)) != 1)
	{
		SetErrorMessage("Error writing to Pi-Cubes I/O module.");
		return;
	}

	// Send address and read value from I/O module
	if ((read(fd, buf, 20)) != 20)
	{
		SetErrorMessage("Error reading from Pi-Cubes I/O module.");
		return;
	}

	// Result Bytes should be this: [0,16,H1,L1,H2,L2,H3,L3,H4,L4,H5,L5,H6,L6,H7,L7,H8,L8,crcHIGH,crcLOW]

	// CRC checking. 16 is a preamble byte
	if (buf[1] == 16) {
		for (unsigned int i=0; i<16; i++ ) {
			inputBytes[i] = buf[i+2];
		}
		//received CRC from Result Bytes, 2 last bytes.
		inputCRC = buf[18] << 8 | buf[19];
		//Calculate CRC from Result bytes, HIGH and LOW. H1-L1 -- to -- H8-L8
		calcCRC = CRC16(inputBytes,16);
	} else {
		return;
	}

	// Good CRC equals 0. means bytes are good. So, we can merge HIGH and LOW bytes to have WORDS 0-1023
	if ((calcCRC - inputCRC) == 0){
		results[0] = buf[2] << 8 | buf[3];
		results[1] = buf[4] << 8 | buf[5];
		results[2] = buf[6] << 8 | buf[7];
		results[3] = buf[8] << 8 | buf[9];
		results[4] = buf[10]<< 8 | buf[11];
		results[5] = buf[12]<< 8 | buf[13];
		results[6] = buf[14]<< 8 | buf[15];
		results[7] = buf[16]<< 8 | buf[17];
	} else {
		return;
	}

	// Close Port
	close(fd);
  }

  // Executed when the async work is complete
  // this function will be run inside the main event loop
  // so it is safe to use V8 again
  void HandleOKCallback () {
    HandleScope scope;

    //Local<Value> argv[] = { Null(), New<Number>(value) };

    Local<Array> array = New<Array>(8);

    for (unsigned int i=0; i<8; i++ ) {
      array->Set(i, New<Number>(results[i]));
    }

    Local<Value> argv[] = { Null(), array };

    callback->Call(2, argv);
  }

 private:
  int module;
  int input;
  int typeui;
  int32_t value;
  unsigned int results[8];
};

// Asynchronous readUI
NAN_METHOD(readUI) {
  int module = To<int>(info[0]).FromJust();
  int input = To<int>(info[1]).FromJust();
  int typeui = To<int>(info[2]).FromJust();
  Callback *callback = new Callback(info[3].As<Function>());

  if (info.Length() != 4)
  {
    return Nan::ThrowError("Incorect number of parameters passed to readUI function !");
  }

  AsyncQueueWorker(new readUIWorker(callback, module, input, typeui));
}

//CRC checking for Synchronous read
unsigned int CRC16(unsigned char *buffer, unsigned char messageLength) {
	unsigned int crc = 0xFFFF;
	unsigned int crcHigh = 0;
	unsigned int crcLow = 0;
	int i, j = 0;

	for (i = 0;i < messageLength;i++)
	{
		crc ^= buffer[i];
		for (j = 8; j != 0; j--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	//bytes are wrong way round so doing a swap here..
	crcHigh = (crc & 0x00FF) << 8;
	crcLow = (crc & 0xFF00) >> 8;
	crcHigh |= crcLow;
	crc = crcHigh;
	return crc;
}

// Synchronous readUI
NAN_METHOD(readUISync) {
	int module = To<int>(info[0]).FromJust();
	int input = To<int>(info[1]).FromJust();
	int typeui = To<int>(info[2]).FromJust();
    unsigned int results[8];
 	unsigned char buf[20];
 	unsigned char inputBytes[16];
 	unsigned int calcCRC = 0;
 	unsigned int inputCRC = 0;
    int fd;
    Local<Array> array = New<Array>(8);

    // Open port for reading and writing
    if ((fd = open("/dev/i2c-1", O_RDWR)) < 0)
    {
		return;
    }

	 // Set the port options and set the address of the device we wish to speak to
	if (ioctl(fd, I2C_SLAVE, 0x12) < 0)
	{
		return;
	}

	// calculate input register address
	buf[0] = 0xAA;

	// Send register address and value to I/O module
	if ((write(fd, buf, 1)) != 1)
	{
		return;
	}

	// Read values from I/O module
	if ((read(fd, buf, 20)) != 20)
	{
		return;
	}

	// Result Bytes should be this: [0,16,H1,L1,H2,L2,H3,L3,H4,L4,H5,L5,H6,L6,H7,L7,H8,L8,crcHIGH,crcLOW]

	// CRC checking. 16 is a preamble byte
	if (buf[1] == 16) {
		for (unsigned int i=0; i<16; i++ ) {
			inputBytes[i] = buf[i+2];
		}
		//received CRC from Result Bytes, 2 last bytes.
		inputCRC = buf[18] << 8 | buf[19];
		//Calculate CRC from Result bytes, HIGH and LOW. H1-L1 -- to -- H8-L8
		calcCRC = CRC16(inputBytes,16);
	} else {
		return;
	}

	// Good CRC equals 0. means bytes are good. So, we can merge HIGH and LOW bytes to have WORDS 0-1023
	if ((calcCRC - inputCRC) == 0){
		results[0] = buf[2] << 8 | buf[3];
		results[1] = buf[4] << 8 | buf[5];
		results[2] = buf[6] << 8 | buf[7];
		results[3] = buf[8] << 8 | buf[9];
		results[4] = buf[10]<< 8 | buf[11];
		results[5] = buf[12]<< 8 | buf[13];
		results[6] = buf[14]<< 8 | buf[15];
		results[7] = buf[16]<< 8 | buf[17];
	} else {
		return;
	}

	for (unsigned int i=0; i<8; i++ ) {
		array->Set(i, New<Number>(results[i]));
	}

	info.GetReturnValue().Set(array);

}
