#include <stdio.h>   // printf
#include <stdlib.h>  // exit
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#ifndef _INCLUDED_PIEZO_SONIC_H_
#define _INCLUDED_PIEZO_SONIC_H_

#define CTRL_IN  LIBUSB_ENDPOINT_IN|LIBUSB_RECIPIENT_INTERFACE     // 0x80 + 0x01
#define CTRL_OUT LIBUSB_ENDPOINT_OUT|LIBUSB_RECIPIENT_INTERFACE    // 0x00 + 0x01

#define MOTOR_NUMS 8

class PiezoSonic
{
	libusb_context *context;
	int dev_num;
	libusb_device **dev_list;

	int id;
	int rotate;
	int speed;
	bool inversion = false;

	libusb_device_handle *handle;
	
	const uint16_t VENDOR_ID  = 0x04d8; // ベンダID
	const uint16_t PRODUCT_ID = 0xedc3; // プロダクトID
	const int PACKET_CTRL_LEN = 64;
	const int TIMEOUT = 5000; /* timeout in ms */  
	
	uint16_t read(uint8_t _address);
	void write(uint8_t _address, uint16_t value);

public:
	PiezoSonic() {};

	void open(int _device_id);
	void close();
	int move(int _speed);
	void stop();
	uint16_t status(int _address);
	void config(int _address, int value);
	void invert();
};

#endif
