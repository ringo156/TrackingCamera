/* Servo.h */
#ifndef __SERVO_H__
#define __SERVO_H__

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdint.h>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "Fuzzy_tools.h"
using namespace boost;
using namespace std;

class Servo{
	private:
		Fuzzy fuzzy;
		void *lwServo_base;
		int fd;
		volatile uint16_t *lwServo_addr;
		double nowAngle[4];
		double oldAngle[4];
		int enable_servo_flag;
		int enable_loop_flag;
		int servo_count;
		int control_mode;
		cv::Point2f nowPoint;
		cv::Point2f targetPoint;
		cv::Point2f devPoint;
		cv::Point2f devPoint_sum;
		cv::Point2f devPoint_d;
		cv::Point2f devPoint_log;
		cv::Point2f pData;

		double kp;
		double ki;
		double kd;
		thread* thr_tracking_loop;
		struct timespec treq,trem;
		enum{
			SERVO_X,
			SERVO_Y
		};
	public:
		Servo();
		~Servo();
		void setPeriod(int num ,int period);
		void setCompare(int num,int cmp);
		void setDivider(int div);
		void angle(int num,double angle);
		void init(int initangle1,int initangle2);
		void tracking_loop();
		void tracking(int ena_flag,cv::Point nowPoint,cv::Point targetPoint,double p,double i,double d, int mode);
};

#endif // __SERVO_H__
