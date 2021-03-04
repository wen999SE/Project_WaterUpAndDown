#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include "V4L2VideoCapture.h"
#include <pthread.h>
#include <QTimer>
#include<semaphore.h>
#include <stdio.h>  
#include "opencv2/core.hpp"  
#include "opencv2/core/utility.hpp"  
#include "opencv2/core/ocl.hpp"  
#include "opencv2/imgcodecs.hpp"  
#include "opencv2/highgui.hpp"  
#include "opencv2/features2d.hpp"  
#include "opencv2/calib3d/calib3d_c.h"  
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"  
#include "opencv2/flann.hpp"  
#include "opencv2/xfeatures2d.hpp"  
#include "opencv2/ml.hpp"  
#include <time.h>
#include <sys/stat.h>

#define IMAGEWIDTH 640
#define IMAGEHEIGHT 480
//不发出debug信息
 #define Release 
 #define debug_2

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace cv::ml;


typedef struct
{
	Point2f left_top;
	Point2f left_bottom;
	Point2f right_top;
	Point2f right_bottom;
} four_corners_t;

typedef enum 
{
    Mode_3Video_Play,
    Mode_Splice_Play
}ChangeMode_Type;

void *Pt_Water_Thread(void* args);
void *Pt_Mid_Thread(void* args); 
void *Pt_MidDown_Thread(void* args);


void CalcCorners(Mat& H, const Mat& src);
void writeMatToXML(const cv::Mat & mat, const std::string xmlName);
void OptimizeSeam(Mat &img1, Mat &trans, Mat &dst);

int V4L2Capture_Calibration_Up(cv::Mat &img, cv::Mat &Calibration_Img, Rect &ROIRect);
int V4L2Capture_Calibration_Water(cv::Mat &img, cv::Mat &Calibration_Img, Rect &ROIRect);

 
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int CameraInit();
    void Pt_Water_ThreadCallBack(void);
    void Pt_Mid_ThreadCallBack(void); 
    void Pt_Up_ThreadCallBack(void); 
    void Pt_MidDown_ThreadCallBack(void);

    // static void *Pt_Water_Thread(void* args)
    static void *Pt_Up_Thread(void* args)
    {
        MainWindow *ptr_MainWindow = (MainWindow *)args;
        ptr_MainWindow->Pt_Up_ThreadCallBack();
        return NULL;
    }

    // static void *Pt_Up_Thread(void* args); 

private slots:
    void on_Change_pushButton_clicked();
    void Video3_Play(void);
    void Splice_Play(void);
signals:
    void captured_image( void );
    void MergeFinish( void );
    

private:
    Ui::MainWindow *ui;
    QTimer *timer_Mode1;
    QTimer *timer_Mode2;




};



#endif // MAINWINDOW_H
