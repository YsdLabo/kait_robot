#include "piezo_sonic.h"

PiezoSonic::PiezoSonic(int _device_id)
{
	id = _device_id;
}

void PiezoSonic::open()
{
	// libusbの初期化
	int ret = libusb_init(&context); 
	if (ret < 0) { 
		printf("[PiezoSonic] Failed to initialize libusb\n"); 
		exit(-1); 
	}
	printf("[PiezoSonic] libusb initialized.\n");    

	// list up usb devices    
	dev_num = libusb_get_device_list(context, &dev_list);
    
	// count the number of usb devices
	//printf("[PiezoSonic] number of usb devices = %d\n", dev_num);

	// PiezoSonicドライバの検索
	if(dev_list != NULL)
	{
		//libusb_device_handle  *dummy_handle = NULL;

		for(int i = 0; i < dev_num; i++) {
			struct libusb_device_descriptor desc;
			libusb_get_device_descriptor(dev_list[i], &desc);

			if(desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID) {
				//printf("[PiezoSonic] find device %d [%04x:%04x]\n", i, desc.idVendor, desc.idProduct);

				handle = NULL;
				if(libusb_open(dev_list[i], &handle) >= 0) {
					if(libusb_kernel_driver_active(handle, 0) != 0) {    // active:1  deactive:0
						libusb_detach_kernel_driver(handle, 0);
						printf("[PiezoSonic] libusb_kernel_driver_active\n");
					}
			
					// Read DIP #1/#2
					unsigned char x = (unsigned char)(read(0x2A) & 0xF0);
					int tmp_id;
					if(x == 0) tmp_id = 0;
					else if(x == 0x80) tmp_id = 1;
					else if(x == 0x40) tmp_id = 2;
					else if(x == 0xC0) tmp_id = 3;
					else if(x == 0x20) tmp_id = 4;
					else if(x == 0xA0) tmp_id = 5;
					else if(x == 0x60) tmp_id = 6;
					else if(x == 0xE0) tmp_id = 7;
					//if(tmp_id >= 0) printf("[PiezoSonic] find piezo sonic motor No : %d\n", tmp_id + 1);

					if(id == tmp_id) {
					 	// USB デバイスの初期化(configration)
						//libusb_set_configuration(handle, 1); 
				
						// インタフェースの要求(デバイスの使用権を要求)
						libusb_claim_interface(handle, 0);
			
						if(id >= 0) printf("[PiezoSonic] open piezo sonic motor No : %d\n", id + 1);
						
						libusb_free_device_list(dev_list, 1);
					
						return;
					}
					libusb_close(handle);
				}
				else printf("open failed\n");
			}
		}
		handle = NULL;
	}
}

void PiezoSonic::close()
{
	if(handle != NULL) {
		libusb_release_interface(handle, 0);   // libusb_claim_interfaceで要求したインタフェースを解放する。デバイスハンドルを閉じる前に解放する必要がある
		libusb_close(handle);
		handle = NULL;
		printf("[PiezoSonic] close piezo sonic motor : %d\n", id + 1); 
		libusb_exit(context); 
		printf("[PiezoSonic] exit\n");
	}
}

int PiezoSonic::move(int _speed)
{
	uint16_t buf = 0;
	int speed = _speed;

	if(handle != NULL) {
		// Write Control 1 register
		write(0x34, 0xE000);	// USB enable, Encoder not use

		// Write Control 2 register
		if(speed == 0) {
			write(0x36, 0);
		}
		else {
			if(speed < 0) {
				buf = 0x2000;	// CCW
				speed *= -1;
			}
			else buf = 0x4000;	// CW
			if(speed > 0x0FFF) speed = 0x0FFF;
			buf |= speed & 0x0FFF;
			write(0x36, buf);
		}
		return 0;
   	}
	return -1;
}

void PiezoSonic::stop()
{
	move(0);
}

uint16_t PiezoSonic::status(int _address)
{
	return read(_address);
}

void PiezoSonic::config(int _address, int _value)
{
	write(_address, _value);
}

uint16_t PiezoSonic::read(uint8_t _address)
{
	if(handle != NULL) {
		unsigned char buf[PACKET_CTRL_LEN] = {0}; 
		unsigned char rcv[PACKET_CTRL_LEN] = {0};
		int data_len;

		// set i2c address : 0x2A
		buf[0] = 0x20;
		buf[2] = _address;
		libusb_interrupt_transfer(handle, CTRL_OUT, buf, PACKET_CTRL_LEN, &data_len, TIMEOUT);
		//libusb_interrupt_transfer(handle, CTRL_IN, rcv, PACKET_CTRL_LEN, &data_len, TIMEOUT);
    		
		// read i2c
		buf[0] = 0x21;
		buf[2] = 0;
		libusb_interrupt_transfer(handle, CTRL_OUT, buf, PACKET_CTRL_LEN, &data_len, TIMEOUT);
		libusb_interrupt_transfer(handle, CTRL_IN, rcv, PACKET_CTRL_LEN, &data_len, TIMEOUT);

		uint16_t tmp = rcv[3];
		tmp <<= 8;
		tmp |= rcv[2];
	
		return tmp;
	}
	return 0;
}

void PiezoSonic::write(uint8_t _address, uint16_t _value)
{
	if(handle != NULL) {
		unsigned char buf[PACKET_CTRL_LEN] = {0};
		unsigned char rcv[PACKET_CTRL_LEN] = {0};
		int data_len;

		// set i2c address : 0x34
		buf[0] = 0x20;
		buf[2] = _address;
		libusb_interrupt_transfer(handle, CTRL_OUT, buf, PACKET_CTRL_LEN, &data_len, TIMEOUT);
		//libusb_interrupt_transfer(handle, CTRL_IN, rcv, PACKET_CTRL_LEN, &data_len, TIMEOUT);

   		// write i2c
		buf[0] = 0x22;
		buf[2] = (unsigned char)(_value & 0x00FF);         // lower
		buf[3] = (unsigned char)((_value & 0xFF00) >> 8);  // upper
		libusb_interrupt_transfer(handle, CTRL_OUT, buf, PACKET_CTRL_LEN, &data_len, TIMEOUT);
		//libusb_interrupt_transfer(handle, CTRL_IN, rcv, PACKET_CTRL_LEN, &data_len, TIMEOUT);
	}
}





