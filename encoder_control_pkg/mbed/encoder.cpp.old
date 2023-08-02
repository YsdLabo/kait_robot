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
	DigitalIn(p7, PullNone),
	DigitalIn(p8, PullNone),

	DigitalIn(p11, PullNone),
	DigitalIn(p12, PullNone),

	DigitalIn(p15, PullNone),
	DigitalIn(p16, PullNone),

	DigitalIn(p17, PullNone),
	DigitalIn(p18, PullNone),

	DigitalIn(p19, PullNone),
	DigitalIn(p20, PullNone),

	DigitalIn(p26, PullNone),
	DigitalIn(p25, PullNone),

	DigitalIn(p24, PullNone),
	DigitalIn(p23, PullNone),

	DigitalIn(p22, PullNone),
	DigitalIn(p21, PullNone)
};

//DigitalOut led1 = LED1;

Ticker  count_ticker;

char encAB[MOTOR_NUMS];
char encAB_old[MOTOR_NUMS];
int enc_cnt[MOTOR_NUMS] = {0};

void enc_loop()
{
	char res;
	for(int i=0;i<MOTOR_NUMS;i++) {
		int ch = 2 * i;
		encAB[i] = din[ch+1];
		encAB[i] <<= 1;
		encAB[i] |= din[ch];
		if(encAB[i] != encAB_old[i]) {
			res = (encAB[i] + (encAB_old[i] << 1)) & 0x03;
			if(res >= 2) enc_cnt[i]++;
			else enc_cnt[i]--;
			encAB_old[i] = encAB[i];
		}
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
		encAB_old[i] = din[ch+1];
		encAB_old[i] <<= 1;
		encAB_old[i] |= din[ch];
		enc_cnt[i] = 0;
	}

	count_ticker.attach(&enc_loop, 0.00002); //org0.00002

    while (1) {
        //led1 = !led1;

        for(int i=0;i<MOTOR_NUMS;i++) {
			enc_count_msg.data[i] = enc_cnt[i];
			enc_cnt[i] = 0;
		}
        pub.publish( &enc_count_msg );
        nh.spinOnce();
        wait_ms(10);
    }
}

