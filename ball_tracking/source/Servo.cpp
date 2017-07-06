#include "Servo.h"

Servo::Servo(){
	if((fd = open("/dev/mem",(O_RDWR | O_SYNC))) == -1){
		printf("ERROR: could not open \"/dev/mem\"...\n");
		return;
	}
	lwServo_base = mmap(NULL,32,(PROT_READ | PROT_WRITE),MAP_SHARED,fd,0xFF200000);
	lwServo_addr = (volatile uint16_t*)(lwServo_base+0x80);
	if(lwServo_base == MAP_FAILED){
		printf("ERROR: mmap() failed...\n");
		close(fd);
		return;
	}
	nowAngle[0] = 120;
	nowAngle[1] = 120;
	nowAngle[2] = 120;
	nowAngle[3] = 120;
	treq.tv_sec = (time_t)0;
	treq.tv_nsec = 10000000;
	enable_loop_flag = 1;
}
Servo::~Servo(){
	enable_loop_flag = 0;
	thr_tracking_loop->join();

	if(munmap(lwServo_base,32) != 0){
		printf("ERROR: mumap() failed...\n");
	}
	close(fd);
}
void Servo::setPeriod(int num ,int period){
	if(num > 3){
		return;
	}
	*(lwServo_addr+num) = period;
}
void Servo::setCompare(int num,int cmp){
	if(num > 3){
		return;
	}
	*(lwServo_addr+4+num) = cmp;
}
void Servo::setDivider(int div){
	*(lwServo_addr+8) = div;
}
void Servo::angle(int num,double angle){
	oldAngle[num] = nowAngle[num];
	nowAngle[num] = angle;
	setDivider(2);
	setPeriod(num,37500);
	setCompare(num,(int)(74.074*angle+8750));
}
void Servo::init(int initangle1,int initangle2){
	setCompare(SERVO_X,0);
	setCompare(SERVO_Y,0);
	sleep(1);
	angle(SERVO_X,initangle1);
	angle(SERVO_Y,initangle2);
	//4777
	//thr_tracking_loop(bind(function<void()>(&Servo::tracking_loop),this));
	thread* thr_tracking_loop = new thread(bind(&Servo::tracking_loop,this));
}
void Servo::tracking_loop(){
	//oldAngle[SERVO_X] = nowAngle[SERVO_X];
	//oldAngle[SERVO_Y] = nowAngle[SERVO_Y];
	while(enable_loop_flag){
		if(enable_servo_flag == 1){
			enable_servo_flag = 0;
			if((nowPoint.x == -1) || (nowPoint.y == -1)){
				return;
			}
			devPoint = nowPoint - targetPoint;//差の計算
			double res = cv::norm(devPoint);
			kp = fuzzy.toKP(res);//kpの値を更新する
			/*
			if((devPoint_log-devPoint_d).inside(cv::Rect(-320,-240,320,240))){
				devPoint_d = devPoint_log - devPoint_d;
			}
			*/
			devPoint_sum += devPoint;
			cv::Point2f pData = devPoint * kp + devPoint_sum * ki + devPoint_d * kd;
			devPoint_log = devPoint;
			if((nowAngle[SERVO_X]+pData.x  < 200) && (nowAngle[SERVO_X]+pData.x > 40)){
				nowAngle[SERVO_X] += pData.x;
			}
			if((nowAngle[SERVO_Y]+pData.y  < 240) && (nowAngle[SERVO_Y]+pData.y > 120)){
				nowAngle[SERVO_Y] += pData.y;
			}

			angle(SERVO_X,nowAngle[SERVO_X]);
			angle(SERVO_Y,nowAngle[SERVO_Y]);

			nanosleep(&treq,&trem);
		}
	}
	return;
}
void Servo::tracking(int ena_flag,cv::Point argNowPoint,cv::Point argTargetPoint,double argp,double argi,double argd){
	nowPoint = argNowPoint;
	targetPoint = argTargetPoint;
	servo_count = 0;
	kp = argp;
	ki = argi;
	kd = argd;
	enable_servo_flag = ena_flag;
}
