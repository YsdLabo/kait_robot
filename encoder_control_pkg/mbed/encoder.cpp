/*
 * 
 * Prints "hello world!"
 */

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include"mbed.h"
#pragma GCC diagnostic warning "-Wdeprecated-declarations"
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

#define MOTOR_NUMS  8

ros::NodeHandle  nh;

std_msgs::Int32MultiArray enc_count_msg;
ros::Publisher pub("/kait_robot/encoder_count", &enc_count_msg);

DigitalIn din[] = {
	DigitalIn(p7, PullNone),    // ch0
	DigitalIn(p8, PullNone),    // ch1

	DigitalIn(p11, PullNone),   // ch2
	DigitalIn(p12, PullNone),   // ch3

	DigitalIn(p15, PullNone),   // ch4
	DigitalIn(p16, PullNone),   // ch5

	DigitalIn(p17, PullNone),   // ch6
	DigitalIn(p18, PullNone),   // ch7

	DigitalIn(p19, PullNone),   // ch8
	DigitalIn(p20, PullNone),   // ch9

	DigitalIn(p26, PullNone),   // ch10
	DigitalIn(p25, PullNone),   // ch11

	DigitalIn(p24, PullNone),   // ch12
	DigitalIn(p23, PullNone),   // ch13

	DigitalIn(p22, PullNone),   // ch14
	DigitalIn(p21, PullNone)    // ch15
};

//DigitalOut led1 = LED1;

Ticker  count_ticker;

char encAB[MOTOR_NUMS];
char encAB_old[MOTOR_NUMS];
int enc_cnt[MOTOR_NUMS] = {0};
static const int encode_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void enc_loop()
{
	for(int i=0;i<MOTOR_NUMS;i++) {
		int ch = 2 * i;
		encAB[i] = (din[ch+1]<<1) | din[ch];
		unsigned char state = (encAB_old[i]<<2) | encAB[i];
		encAB_old[i] = encAB[i];
		enc_cnt[i] += encode_table[state];
	}
}

int main() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub);

	enc_count_msg.data = (int32_t*)malloc(sizeof(int32_t) * MOTOR_NUMS);
	enc_count_msg.data_length = MOTOR_NUMS;

	for(int i=0;i<MOTOR_NUMS;i++) {
		int ch = 2 * i;
		encAB_old[i] = (din[ch+1]<<1) | din[ch];
		enc_cnt[i] = 0;
	}

	count_ticker.attach(&enc_loop, 0.00002); //org0.00002

    while (1) {
        //led1 = !led1;

        for(int i=0;i<MOTOR_NUMS;i++) {
			enc_count_msg.data[i] = enc_cnt[i];
			enc_cnt[i] = 0;
		}
		__disable_irq();
        pub.publish( &enc_count_msg );
        nh.spinOnce();
        __enable_irq();

        wait_ms(5);
    }
}

