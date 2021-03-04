#ifndef _V4L2VIDEOCAPTURE_H_
#define _V4L2VIDEOCAPTURE_H_

#include <unistd.h>
#include <error.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <opencv2/highgui/highgui.hpp>

using namespace std;

#define CLEAR(x) memset(&(x), 0, sizeof(x))

int kbhit(void);

class V4L2Capture
{
public:
	V4L2Capture(char *devName, int width, int height/*, cv::Mat &cameraMatrix, cv::Mat &distCoeffs*/);
	virtual ~V4L2Capture();

	int openDevice();
	int closeDevice();
	int initDevice();
	int startCapture();
	int set_exposure(int value);
	int stopCapture();
	int freeBuffers();
	int getFrame(void **,size_t *);
	int backFrame();
	int V4L2Capture_Calibration(cv::Mat &img, cv::Mat &Calibration_Img);

	V4L2Capture& operator >> (cv::Mat & image);

	IplImage* Capture_IplImage;

private:
	int initBuffers();

	struct cam_buffer
	{
		void* start;
		unsigned int length;
	};
	char *devName;
	int capW;
	int capH;
	int fd_cam;
	cam_buffer *buffers;
	unsigned int n_buffers;
	int frameIndex;
	
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;


	unsigned char *yuv422frame;
	unsigned long yuvframeSize;
};


#endif

