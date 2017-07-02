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

class Camera{
	private:
		void* camera_base;
		void* lwSdram_base;
		void* lwCameraIp_base;
		void* lwCameraIpRead_base;
		int fd;
		int fd_camera;
		volatile uint8_t *camera_addr;
		volatile uint16_t *lwSdram_addr;
		volatile uint32_t *lwCameraIp_addr;
		volatile uint32_t *lwCameraIpRead_addr;
	public:
		enum{
			BUTTON_DOWN,
			BUTTON_UP
		};

		int lbuttoon_flag;
		cv::Mat camera_mt;
		cv::Mat camera_bin;
		cv::Vec2d push_point;

		Camera();
		~Camera();
		void shutterRelease();
		void modeChange(int mode);
		void threshold(unsigned int th);
		void hue_threshold(unsigned int th);
		void hue_range(unsigned int th);
		void sat_threshold_low(unsigned int th);
		void sat_threshold_high(unsigned int th);
		void light_threshold_low(unsigned int th);
		void light_threshold_high(unsigned int th);
		int getStatus();
		bool shutterWait();
		cv::Mat getImg(int cloneFlag);
};

