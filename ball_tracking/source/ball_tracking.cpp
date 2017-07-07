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
#include "Labeling.h"
#include "Servo.h"
#include "Camera.h"

using namespace std;

class BallDetect{
	private:
		cv::Mat element;
		int median_size;
		int morpho_n;
		int morpho;
		LabelingBS label;
	public:
		BallDetect(){

		}
		void setting(int argMedian_size,int argMorpho_n,
				int argMorpho=cv::MORPH_ERODE,
				int argMorho_x = 3,int argMorho_y = 3){
			median_size = argMedian_size;
			morpho_n = argMorpho_n;
			morpho = argMorpho;
			element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(argMorho_y,argMorho_x));
		}
		cv::Mat extraction(cv::Mat binImg){
			if(median_size != 0){
				cv::medianBlur(binImg,binImg,median_size);
			}
			for(int i;i < morpho_n;i++){
				cv::morphologyEx(binImg,binImg,morpho,element);
			}
			return binImg;
		}
		void labeling(cv::Mat binImg,int thSize1,int thSize2,
			cv::Point &center,cv::Point &point1,cv::Point &point2){
			cv::Mat labelImage(binImg.size(),CV_16SC1);
			label.Exec(binImg.data,(short*)labelImage.data,binImg.cols,binImg.rows,false,thSize1);
			center.x = -1;
			center.y = -1;
			for (int i = 0; i < label.GetNumOfResultRegions(); i++)
			{
				RegionInfoBS *regioninfo = label.GetResultRegionInfo(i);
				int x1, x2, y1, y2, w, h;
				regioninfo->GetMin(x1, y1);
				regioninfo->GetMax(x2, y2);
				w = x2 - x1;
				h = y2 - y1;
				float x, y;
				regioninfo->GetCenter(x, y);
				int center_x = (int)x;
				int center_y = (int)y;

				if((w > thSize2) && (h > thSize2)){
					point1.x = x1;
					point1.y = y1;
					point2.x = x2;
					point2.y = y2;
					center.x = center_x;
					center.y = center_y;
					return;
				}
			}
		}
		cv::Mat labeling_hard(cv::Mat binImg,int *max_y_line,int *maxValue,int *max_x_line){
			int sumValue;
			cv::Vec4b pixData;
			int maxValueData = 0;
			int max_y_lineData;
			int max_x_lineData;
			long center_gravity_sum = 0;
			long center_gravity_ave = 1;
			int center_gravity_x;
			int center_gravity_y;
			cv::Mat result_img = binImg.clone();
			for(int i = 0;i < 470;i++){
				pixData = binImg.at<cv::Vec4b>(i,638);
				sumValue = ((pixData[0] & 0xff) + ((pixData[1] << 8) & 0xff00));
				sumValue /= 2;
				if((sumValue  > maxValueData) && (sumValue < 640)){
					maxValueData = sumValue;
					max_y_lineData = i;
				}
				else{
				}
				if(sumValue > 640)
				{
					sumValue = 640;
				}
				else {
					//center_gravity_sum += i * sumValue;
					//center_gravity_ave += sumValue;
				}
			}
			if(maxValueData > 30){
				*maxValue = maxValueData;
			}
			//center_gravity_y = center_gravity_sum / center_gravity_ave;
			center_gravity_sum = 0;
			center_gravity_ave = 1;
			for(int i = 30;i < 630;i++){
				pixData = binImg.at<cv::Vec4b>(477,i);
				sumValue = ((pixData[0] & 0xff) + ((pixData[1] << 8) & 0xff00));
				sumValue /= 2;
				if((sumValue  > maxValueData) && (sumValue < 480)){
					maxValueData = sumValue;
					max_x_lineData = i;
				}
				else{
				}
				if(sumValue > 480)
				{
					sumValue = 480;
				}
				else{
					//center_gravity_sum += i * sumValue;
					//center_gravity_ave += sumValue;
				}
			}
			//center_gravity_x = center_gravity_sum / center_gravity_ave;
			*max_y_line = max_y_lineData;
			*max_x_line = max_x_lineData;
			//*max_y_line = center_gravity_y;
			//*max_x_line = center_gravity_x;
			return result_img;
		}
		cv::Mat labeling_hard_hist(cv::Mat binImg,int *max_y_line,int *maxValue,int *max_x_line){
			int sumValue;
			cv::Vec4b pixData;
			int maxValueData = 0;
			int max_y_lineData;
			int max_x_lineData;
			long center_gravity_sum = 0;
			long center_gravity_ave = 1;
			int center_gravity_x;
			int center_gravity_y;
			cv::Mat result_img = binImg.clone();
			for(int i = 0;i < 470;i++){
				pixData = binImg.at<cv::Vec4b>(i,638);
				sumValue = ((pixData[0] & 0xff) + ((pixData[1] << 8) & 0xff00));
				sumValue /= 2;
				if((sumValue  > maxValueData) && (sumValue < 640)){
				//if((sumValue  > maxValueData)){
				//if((sumValue  > maxValueData)){
					maxValueData = sumValue;
					max_y_lineData = i;
				}
				else{
				}
				if(sumValue > 640)
				{
					sumValue = 640;
				}
				if(sumValue != 0){
					center_gravity_sum += i * sumValue;
					center_gravity_ave += sumValue;
					cv::line(result_img,cv::Point(640-sumValue,i),cv::Point(638,i),cv::Scalar(255,0,0));
				}
			}
			if(maxValueData > 30){
				*maxValue = maxValueData;
			}
			center_gravity_y = center_gravity_sum / center_gravity_ave;
			center_gravity_sum = 0;
			center_gravity_ave = 1;
			maxValueData = 0;
			for(int i = 30;i < 630;i++){
				pixData = binImg.at<cv::Vec4b>(477,i);
				sumValue = ((pixData[0] & 0xff) + ((pixData[1] << 8) & 0xff00));
				sumValue /= 2;
				if((sumValue  > maxValueData) && (sumValue < 480)){
				//if((sumValue  > maxValueData)){
				//if((sumValue  > maxValueData)){
					maxValueData = sumValue;
					max_x_lineData = i;
				}
				else{
				}
				if(sumValue > 480)
				{
					sumValue = 480;
				}
				if(sumValue != 0){
					cv::line(result_img,cv::Point(i,480-sumValue),cv::Point(i,479),cv::Scalar(0,255,0));
					center_gravity_sum += i * sumValue;
					center_gravity_ave += sumValue;
				}
			}
			center_gravity_x = center_gravity_sum / center_gravity_ave;
			//*max_y_line = max_y_lineData;
			//*max_x_line = max_x_lineData;
			*max_y_line = center_gravity_y;
			*max_x_line = center_gravity_x;
			return result_img;
		}
};

class BTS_param{
	public:
		int mode;
		int image_reduction_flag;
		int image_not_flag;
		int hue_threshold;
		int hue_range;
		int sat_threshold_low;
		int sat_threshold_high;
		int light_threshold_low;
		int light_threshold_high;
		int gray_threshold;
		int morpho_n;
		int median_size;
		int set_param_flag;
		int set_mode_flag;
		int set_camera_clone_flag;
		int set_extraction_flag;
		int set_soft_labering_flag;
		int set_hard_labering_flag;
		int print_mode;
		int threshold_mode;
		int control_mode;
		int hue_max;
		int hue_min;
		int maxValue,max_y_line,max_x_line;
		double kp;
		double ki;
		double kd;
		cv::Mat camera_mt;
		cv::Mat camera_bin;
		cv::Mat camera_hsv;
		void key_get(char k){
			set_param_flag = 1;
			if(k == 255){
				set_param_flag = 0;
				set_mode_flag = 0;
			}
			else if(k == 'a'){
				//camera.lbuttoon_flag = 0;
			}
			else if(k == 'u'){//Threshold UP
				//camera.hue_range(20);
				if(gray_threshold < 255){
					gray_threshold++;
				}
				if(hue_threshold < 255){
					hue_threshold++;
				}
				else {
					hue_threshold = 0;
				}
			}
			else if(k == 'd'){//Threshold Down
				//camera.hue_range(20);
				if(gray_threshold > 0){
					gray_threshold--;
				}
				if(hue_threshold > 0){
					hue_threshold--;
				}
				else {
					hue_threshold = 255;
				}
			}
			else if(k == 'm'){//modeChange
				if(mode < 5)
				{
					mode++;
				}
				else{
					mode = 0;
				}
				set_mode_flag = 1;
			}
			else if(k == 'p'){//Image Drawing
				if(print_mode < 2){
					print_mode++;
				}
				else{
					print_mode = 0;
				}
			}
			else if(k == 's'){//Staturation
				if(sat_threshold_low < 125){
					sat_threshold_low += 10;
				}
				else{
					sat_threshold_low = 0;
				}
			}
			else if(k == 'v'){//Value(明度)
				if(light_threshold_low < 205){
					light_threshold_low += 50;
				}
				else{
					light_threshold_low = 0;
				}
			}
			else if(k == 'c'){//pidの値変更
				printf("kp = ");
				cin >> kp;
				printf("ki = ");
				cin >> ki;
				printf("kd = ");
				cin >> kd;
			}
			else if(k == 'f'){//pidとFuzzyの切り替え
				if(control_mode < 1){
					control_mode++;
				}
				else{
					control_mode = 0;
				}
			}
			else{
				//camera.lbuttoon_flag = 1;
			}
		}
		void param_set(Camera &camera,BallDetect &ballDet){
			int filter_select = mode;
			hue_range = 20;
			light_threshold_high = 255;
			sat_threshold_high = 255;
			if(set_param_flag == 1){
				camera.hue_threshold(hue_threshold);
				camera.hue_range(hue_range);
				camera.sat_threshold_low(sat_threshold_low);
				camera.sat_threshold_high(sat_threshold_high);
				camera.light_threshold_low(light_threshold_low);
				camera.light_threshold_high(light_threshold_high);
				camera.threshold(gray_threshold);
				if(hue_threshold + hue_range < 255){
					hue_max = hue_threshold + hue_range;
				}
				else{
					hue_max = hue_threshold + hue_range - 255;
				}
				if(hue_threshold - hue_range > 0){
					hue_min = hue_threshold - hue_range;
				}
				else{
					hue_min = hue_threshold - hue_range + 255;
				}
			}
			if(set_mode_flag == 1){
				if(mode == 0){//RGM
					image_reduction_flag = 1;
					set_camera_clone_flag = 1;
					set_extraction_flag = 1;
					median_size = 7;
					morpho_n = 1;
					set_soft_labering_flag = 1;
					kp = 0.01;
					ki = 0.0001;
					kd = 0.0001;

					threshold_mode = 2;
					image_not_flag = 0;
					set_hard_labering_flag = 0;
				}
				else if(mode == 1){//GrayScale
					image_reduction_flag = 1;
					set_camera_clone_flag = 1;
					set_extraction_flag = 1;
					median_size = 7;
					morpho_n = 1;
					set_soft_labering_flag = 1;
					kp = 0.01;
					ki = 0.0001;
					kd = 0.0001;

					threshold_mode = 1;
					image_not_flag = 0;
					set_hard_labering_flag = 0;
				}
				else if(mode == 2){//Binary
					image_reduction_flag = 1;
					set_camera_clone_flag = 1;
					set_extraction_flag = 1;
					median_size = 7;
					morpho_n = 1;
					set_soft_labering_flag = 1;
					kp = 0.01;
					ki = 0.0001;
					kd = 0.0001;

					threshold_mode = 0;
					image_not_flag = 1;
					set_hard_labering_flag = 0;
				}
				else if(mode == 3){//HSB Binary
					image_reduction_flag = 1;
					set_camera_clone_flag = 1;
					set_extraction_flag = 1;
					median_size = 7;
					morpho_n = 1;
					set_soft_labering_flag = 1;
					kp = 0.01;
					ki = 0.0001;
					kd = 0.0001;

					threshold_mode = 0;
					image_not_flag = 0;
					set_hard_labering_flag = 0;
				}
				else if(mode == 4){//HSV binary + Noise Filter
					image_reduction_flag = 1;
					set_camera_clone_flag = 1;
					set_extraction_flag = 0;
					set_soft_labering_flag = 1;
					kp = 0.01;
					ki = 0.0001;
					kd = 0.0001;

					threshold_mode = 0;
					image_not_flag = 0;
					set_hard_labering_flag = 0;
				}
				else if(mode == 5){//HSV binary + Noise Filter + Image Drawing
					filter_select = 4;
					image_reduction_flag = 0;
					set_camera_clone_flag = 1;
					set_extraction_flag = 0;
					set_soft_labering_flag = 0;
					kp = 0.01;
					ki = 0.0001;
					kd = 0.0001;
					threshold_mode = 0;
					image_not_flag = 0;
					set_hard_labering_flag = 1;
				}
				camera.modeChange(filter_select);
				ballDet.setting(median_size,morpho_n,cv::MORPH_ERODE,1,3);
			}
		}
		void update(Camera &camera,Servo &servo,BallDetect &ballDet){
			cv::Point center,point1,point2;
			camera_mt = camera.getImg(set_camera_clone_flag);
			if(threshold_mode == 0){
				if(image_reduction_flag == 1){
					cv::cvtColor(camera_mt,camera_bin,CV_BGR2GRAY);
					cv::resize(camera_bin,camera_bin,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
					if(image_not_flag){
						camera_bin = ~camera_bin;
					}
				}
			}
			else if(threshold_mode == 1){
				if(image_reduction_flag == 1){
					cv::cvtColor(camera_mt,camera_bin,CV_BGR2GRAY);
					cv::resize(camera_bin,camera_bin,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
				}
				cv::threshold(camera_bin,camera_bin,gray_threshold,0xff,cv::THRESH_BINARY_INV);
			}
			else if(threshold_mode == 2){
				if(image_reduction_flag == 1){
					cv::resize(camera_mt,camera_hsv,cv::Size(),0.5,0.5,cv::INTER_NEAREST);
				}
				cv::cvtColor(camera_hsv,camera_hsv,CV_BGR2HSV);
				cv::inRange(camera_hsv,
						cv::Scalar(hue_min*1.4118 - 180,sat_threshold_low,light_threshold_low,0),
						cv::Scalar(hue_max*1.4118 - 180,sat_threshold_high,light_threshold_high,0),
						camera_bin);
			}
			if(set_extraction_flag == 1){
				camera_bin = ballDet.extraction(camera_bin);
			}
			if(set_hard_labering_flag == 0){
				if(set_soft_labering_flag == 1){
					if(print_mode != 0){
						ballDet.labeling(camera_bin,1000,20,center,point1,point2);
					//cv::rectangle(camera_mt, point1*2, point2*2, cv::Scalar(0, 0, 255));
						servo.tracking(1,
								center,//対象の重心(ターゲット)
								cv::Point(camera_bin.cols*0.5,camera_bin.rows*0.5),//中心
								kp,0,0,control_mode);
					}
					else {
					}
				}
			}
			else if(set_hard_labering_flag == 1){

				//cout << "maxValue = " << maxValue << endl;
				if(print_mode != 0){//Servo detect
					camera_bin
						= ballDet.labeling_hard_hist(camera_mt,&max_y_line,&maxValue,&max_x_line);
					if((max_x_line != 0) && (max_y_line != 0)){
						servo.tracking(1,
								cv::Point(max_x_line,max_y_line),
								cv::Point(camera_bin.cols*0.5,
									camera_bin.rows*0.5),
								kp,ki,kd,control_mode);
						cv::line(camera_bin,cv::Point(0,max_y_line),cv::Point(638,max_y_line),cv::Scalar(255,0,0));
						cv::line(camera_bin,cv::Point(max_x_line,0),cv::Point(max_x_line,478),cv::Scalar(255,0,0));
					}
				}
				else{
				}
			}
			if(print_mode != 2){//Image Draw
				cv::imshow("camera_trans",camera_mt);
				cv::imshow("camera",camera_bin);
			}
			else{
			}
		}
		void init(Camera &camera,Servo &servo,BallDetect &ballDet){
			gray_threshold = 18;
			mode = 0;
			control_mode = 0;
			set_mode_flag = 1;
			camera.shutterRelease();
			servo.init(120,220);
			ballDet.setting(0,0,cv::MORPH_ERODE,1,3);
			hue_threshold = 150;
			hue_range = 30;
			sat_threshold_low = 100;
			sat_threshold_high = 255;
			light_threshold_low = 0;
			light_threshold_high = 255;
			print_mode = 0;
			param_set(camera,ballDet);
		}
};

void mouse_callback(int event,int x,int y,int flags,void* param){
	Camera* camera = static_cast<Camera*>(param);
	if(cv::EVENT_LBUTTONDOWN){
		camera->lbuttoon_flag = 0;
	}
	else {
		camera->lbuttoon_flag = 1;
	}
}

int main(int argc,char **argv)
{
	cv::Mat camera_mt(480,640,CV_8UC4);
	cv::Mat camera_mt_reg(480,640,CV_8UC4);
	cv::Mat hsv_mt(480,640,CV_8UC3);
	cv::Mat hsv_bin;
	cv::Mat hist_bin;

	Camera camera;
	Servo servo;
	BallDetect ballDet;
	BTS_param ballTrack;
	cv::namedWindow("camera",CV_WINDOW_AUTOSIZE);
	double msec;
	cv::Mat msec_log(1,20,CV_64F);
	cv::Mat ave;
	int i = 0;
	//cv::setMouseCallback("camera",mouse_callback,(void*)&camera);

	ballTrack.init(camera,servo,ballDet);
	while(1){
		struct timespec start_val;
		clock_gettime(CLOCK_MONOTONIC,&start_val);

		char k = cv::waitKey(1);
		if(k == 27) break;
		ballTrack.key_get(k);
		ballTrack.param_set(camera,ballDet);
		ballTrack.update(camera,servo,ballDet);

		struct timespec end;
		clock_gettime(CLOCK_MONOTONIC,&end);
		long sec = end.tv_sec - start_val.tv_sec;
		long nsec = end.tv_nsec-start_val.tv_nsec;
		if(nsec < 0){
			sec--;
			nsec += 1000000000L;
		}
		msec = sec*1000+nsec/1000000;
		msec_log.at<double>(0,i) = msec;
		cv::reduce(msec_log,ave,1,CV_REDUCE_AVG);
		msec = ave.at<double>(0,0);
		if(i < 19){
			i++;
		}
		else{
			i = 0;
		}
		printf("time = %6.2fms,hue=%d,gray=%d,mode=%d,control_mode=%d\n"
				,msec
				,ballTrack.hue_threshold
				,ballTrack.gray_threshold
				,ballTrack.mode
				,ballTrack.control_mode);
				//,ballTrack.sat_threshold_low
				//,ballTrack.kp,ballTrack.ki,ballTrack.kd);

	}
	cv::destroyAllWindows();
	return 0;
}
