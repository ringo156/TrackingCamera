#include "Camera.h"
Camera::Camera(){
	if((fd = open("/dev/mem",(O_RDWR | O_SYNC))) == -1){
		printf("ERROR: could not open \"/dev/mem\"...\n");
		return;
	}
	fd_camera = open("/dev/mem",(O_RDONLY));
	lwSdram_base = mmap(NULL,0x10000,(PROT_READ | PROT_WRITE),MAP_SHARED,fd,0xFFC20000);
	lwCameraIp_base = mmap(NULL,0x10000,(PROT_READ | PROT_WRITE),MAP_SHARED,fd,0xFF202000);
	lwCameraIpRead_base = mmap(NULL,0x80,(PROT_READ | PROT_WRITE),MAP_SHARED,fd,0xFF200000);
	camera_base = mmap(NULL,0x960000,(PROT_READ),(MAP_PRIVATE | MAP_LOCKED),fd_camera,0x20000000);
	if(camera_base == MAP_FAILED){
		printf("ERROR: mmap() failed...\n");
		close(fd);
		return;
	}
	camera_addr = (volatile uint8_t*)camera_base;
	lwCameraIp_addr = (volatile uint32_t*)(lwCameraIp_base+1281*4);
	lwCameraIpRead_addr = (volatile uint32_t*)(lwCameraIpRead_base);
	lwSdram_addr = (volatile uint16_t*)(lwSdram_base+0x5080);

	*lwSdram_addr = 0xffff;
}
Camera::~Camera()
{
	munmap(lwSdram_base,0x10000);
	munmap(lwCameraIp_base,0x10000);
	if(munmap(camera_base,0x20000) != 0){
		printf("ERROR: mumap() failed...\n");
	}
	close(fd);
}
void Camera::shutterRelease(){
	*lwCameraIp_addr = 0x0001;
}
void Camera::modeChange(int mode){
	*(lwCameraIp_addr+2) = mode;
}
void Camera::threshold(unsigned int th){
	*(lwCameraIp_addr+3) = th;
}
void Camera::hue_threshold(unsigned int th){
	*(lwCameraIp_addr+4) = th;
}
void Camera::hue_range(unsigned int th){
	*(lwCameraIp_addr+5) = th;
}
void Camera::sat_threshold_low(unsigned int th){
	*(lwCameraIp_addr+7) = th;
}
void Camera::sat_threshold_high(unsigned int th){
	*(lwCameraIp_addr+8) = th;
}
void Camera::light_threshold_low(unsigned int th){
	*(lwCameraIp_addr+9) = th;
}
void Camera::light_threshold_high(unsigned int th){
	*(lwCameraIp_addr+10) = th;
}
int Camera::getStatus(){
	return *(lwCameraIp_addr+1);
}
bool Camera::shutterWait(){
	while((*(lwCameraIp_addr+1))&0x0400 == 0x0000){
		printf("status = %x\n",(*(lwCameraIp_addr+1)));
	}
	while((*(lwCameraIp_addr+1))&0x0400 == 0x0400){
		printf("status = %x\n",(*(lwCameraIp_addr+1)));
	}
	while((*(lwCameraIp_addr+1))&0x0400 == 0x0000){
		printf("status = %x\n",(*(lwCameraIp_addr+1)));
	}
	//while((*(lwCameraIp_addr))&0x0001 == 0x0001);
	return true;
}
cv::Mat Camera::getImg(int cloneFlag){
	uint32_t frame = *(lwCameraIpRead_addr);
	if(frame == 0){
		frame = 7;
	}
	else{
		frame--;
	}
	//cout << "frame = " <<  frame << hex << endl;
	//printf("frame = 0x%x\n",frame);
	//camera_mt = cv::Mat(480,640,CV_8UC4,(void*)(camera_addr+frame*640*480*4));
	camera_mt = cv::Mat(480,640,CV_8UC4,(void*)(camera_addr+frame*640*480*4));
	//cv::cvtColor(camera_mt,camera_bin,CV_BGR2GRAY);
	//cv::resize(camera_bin,camera_bin,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
	//camera_mt = cv::Mat(240,320,CV_8UC4,(void*)(camera_addr+frame*640*480*4));
	if(cloneFlag != 0)
	{
		return camera_mt.clone();

	}
	else{
		return camera_mt;
	}
}

