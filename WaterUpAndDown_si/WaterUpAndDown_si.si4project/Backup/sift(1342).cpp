#include <iostream>  
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
#include <unistd.h>
#include "sift.hpp"

 
#define IMAGEWIDTH 640
#define IMAGEHEIGHT 480


four_corners_t corners;
 

 

void writeMatToXML(const cv::Mat & mat, const std::string xmlName)
{
	FileStorage fs(xmlName, FileStorage::WRITE);
	fs << "Mat" << mat;
	fs.release();
} 
void CalcCorners(const Mat& H, const Mat& src)
{
	double v2[] = { 0, 0, 1 };//左上角
	double v1[3];//变换后的坐标值
	Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
 
	V1 = H * V2;
	//左上角(0,0,1)
	// cout << "V2: " << V2 << endl;
	// cout << "V1: " << V1 << endl;
	corners.left_top.x = v1[0] / v1[2];
	corners.left_top.y = v1[1] / v1[2];
 
	//左下角(0,src.rows,1)
	v2[0] = 0;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.left_bottom.x = v1[0] / v1[2];
	corners.left_bottom.y = v1[1] / v1[2];
 
	//右上角(src.cols,0,1)
	v2[0] = src.cols;
	v2[1] = 0;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.right_top.x = v1[0] / v1[2];
	corners.right_top.y = v1[1] / v1[2];
 
	//右下角(src.cols,src.rows,1)
	v2[0] = src.cols;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.right_bottom.x = v1[0] / v1[2];
	corners.right_bottom.y = v1[1] / v1[2];
 
}
 

int V4L2Capture_Calibration(cv::Mat &img, cv::Mat &Calibration_Img, Rect &ROIRect)
{

	// cv::Mat cameraMatrix;
	// cv::Mat distCoeffs;

	//mo 
    // Mat UpcameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
    // UpcameraMatrix.at<double>(0, 0) = 2.614299995205457e+02;
    // UpcameraMatrix.at<double>(0, 1) = -2.375033225527123;
    // UpcameraMatrix.at<double>(0, 2) = 3.388287344810907e+02;
    // UpcameraMatrix.at<double>(1, 1) = 2.386931028313862e+02;
    // UpcameraMatrix.at<double>(1, 2) = 2.581505697940285e+02;
 
    // Mat UpdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
    // UpdistCoeffs.at<double>(0, 0) = -0.311603035507072; //max
    // UpdistCoeffs.at<double>(1, 0) = 0.056663058487880; // min
	// //calibrationSession.CameraParameters.TangentialDistortion  
    // UpdistCoeffs.at<double>(2, 0) = -0.017004994794971; // max
    // UpdistCoeffs.at<double>(3, 0) = -0.003907738429808; // min
    // UpdistCoeffs.at<double>(4, 0) = 0;


	//可用
 //calibrationSession.CameraParameters.IntrinsicMatrix
    Mat UpcameraMatrix = Mat::eye(3, 3, CV_64F);
	//fx
    UpcameraMatrix.at<double>(0, 0) = 2.506310428377722e+02;
    UpcameraMatrix.at<double>(0, 1) = -2.631754475026844;
	//cx
    UpcameraMatrix.at<double>(0, 2) = 2.944098596676195e+02;
	//fy
    UpcameraMatrix.at<double>(1, 1) = 2.371926140206075e+02;
	//cy
    UpcameraMatrix.at<double>(1, 2) = 2.497785903293827e+02;
 
    Mat UpdistCoeffs = Mat::zeros(5, 1, CV_64F);
	//calibrationSession.CameraParameters.RadialDistortion
    UpdistCoeffs.at<double>(0, 0) = -0.326189051015496; //max
    UpdistCoeffs.at<double>(1, 0) = 0.078045115514857; // min
	//calibrationSession.CameraParameters.TangentialDistortion  
    UpdistCoeffs.at<double>(2, 0) = -0.003442720347599; // max
    UpdistCoeffs.at<double>(3, 0) = 0.010391914805053; // min
    UpdistCoeffs.at<double>(4, 0) = 0;

	//参考
	// //calibrationSession.CameraParameters.IntrinsicMatrix
	// //fx
    // UpcameraMatrix.at<double>(0, 0) = 2.583654223793172e+02;
    // UpcameraMatrix.at<double>(0, 1) = 0.886377263161326;
	// //cx
    // UpcameraMatrix.at<double>(0, 2) = 3.131673407832941e+02;
	// //fy
    // UpcameraMatrix.at<double>(1, 1) = 2.388197299082707e+02;
	// //cy
    // UpcameraMatrix.at<double>(1, 2) = 2.581840768182273e+02;
 
    // Mat UpdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
    // UpdistCoeffs.at<double>(0, 0) = -0.202507168706085; //max
    // UpdistCoeffs.at<double>(1, 0) = 0.031680465407062; // min
	// //calibrationSession.CameraParameters.TangentialDistortion  
    // UpdistCoeffs.at<double>(2, 0) = -0.020028040310166; // max
    // UpdistCoeffs.at<double>(3, 0) = -0.002473946351402; // min
    // UpdistCoeffs.at<double>(4, 0) = 0;




	// Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
    // WatercameraMatrix.at<double>(0, 0) = 2.836571597121089e+02;
    // WatercameraMatrix.at<double>(0, 1) = 0.302994515520648;
    // WatercameraMatrix.at<double>(0, 2) = 3.308421290613233e+02;
    // WatercameraMatrix.at<double>(1, 1) = 2.647412756017969e+02;
    // WatercameraMatrix.at<double>(1, 2) = 7.558505317851999;
 
    // Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
    // WaterdistCoeffs.at<double>(0, 0) = -0.246899590875519; //max
    // WaterdistCoeffs.at<double>(1, 0) = 0.046690851958435; // min
	// //calibrationSession.CameraParameters.TangentialDistortion  
    // WaterdistCoeffs.at<double>(2, 0) = -0.001789654977026; // max
    // WaterdistCoeffs.at<double>(3, 0) = 1.795872231893150e-04; // min
    // WaterdistCoeffs.at<double>(4, 0) = 0;


	// Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
    // WatercameraMatrix.at<double>(0, 0) = 2.783149850391360e+02;
    // WatercameraMatrix.at<double>(0, 1) = 0.298217438206081;
    // WatercameraMatrix.at<double>(0, 2) = 3.439983226605749e+02;
    // WatercameraMatrix.at<double>(1, 1) = 2.614159156300671e+02;
    // WatercameraMatrix.at<double>(1, 2) = 1.180070804488273e+02;
 
    // Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
    // WaterdistCoeffs.at<double>(0, 0) = -0.276180770814349; //max
    // WaterdistCoeffs.at<double>(1, 0) = 0.063194594607371; // min
	// //calibrationSession.CameraParameters.TangentialDistortion  
    // WaterdistCoeffs.at<double>(2, 0) = 0.001224978291421; // max
    // WaterdistCoeffs.at<double>(3, 0) = 5.661827248780201e-04; // min
    // WaterdistCoeffs.at<double>(4, 0) = 0;

    Mat view, rview, map1, map2;
    Size imageSize;
    imageSize = img.size();
	Size OutImageSize(300, 300);
    initUndistortRectifyMap(UpcameraMatrix, UpdistCoeffs, Mat(),
    getOptimalNewCameraMatrix(UpcameraMatrix, UpdistCoeffs, imageSize, 0, imageSize, &ROIRect),
    imageSize, CV_16SC2, map1, map2);

	

	// initUndistortRectifyMap(WatercameraMatrix, WaterdistCoeffs, Mat(),
    // getOptimalNewCameraMatrix(WatercameraMatrix, WaterdistCoeffs, imageSize, 1, imageSize, &ROIRect),
    // imageSize, CV_16SC2, map1, map2);

	
	// fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
    //         getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0), 
	// 		image_size, CV_32FC1, mapx, mapy);

	remap(img, Calibration_Img, map1, map2, INTER_LINEAR);
} 

int V4L2Capture_Calibration2(cv::Mat &img, cv::Mat &Calibration_Img, Rect &ROIRect)
{

	// cv::Mat cameraMatrix;
	// cv::Mat distCoeffs;

	//mo 
    Mat UpcameraMatrix = Mat::eye(3, 3, CV_64F);

	//calibrationSession.CameraParameters.IntrinsicMatrix
    UpcameraMatrix.at<double>(0, 0) = 5.185880932630046e+02;
    UpcameraMatrix.at<double>(0, 1) = 17.336954083655893;
    UpcameraMatrix.at<double>(0, 2) = 3.087822303620704e+02;
    UpcameraMatrix.at<double>(1, 1) = 4.555369092551889e+02;
    UpcameraMatrix.at<double>(1, 2) = 3.423304057754957e+02;
 
    Mat UpdistCoeffs = Mat::zeros(5, 1, CV_64F);
	//calibrationSession.CameraParameters.RadialDistortion
    UpdistCoeffs.at<double>(0, 0) = -0.529187877052290; //max
    UpdistCoeffs.at<double>(1, 0) = 0.105261129928427; // min
	//calibrationSession.CameraParameters.TangentialDistortion  
    UpdistCoeffs.at<double>(2, 0) = -0.061167435606287; // max
    UpdistCoeffs.at<double>(3, 0) = 0.031070402666666; // min
    UpdistCoeffs.at<double>(4, 0) = 0;
	
//*********************************************************************************

	// //可用
 	// //calibrationSession.CameraParameters.IntrinsicMatrix
    // Mat UpcameraMatrix = Mat::eye(3, 3, CV_64F);
	// //fx
    // UpcameraMatrix.at<double>(0, 0) = 2.270455038650727e+02;
    // UpcameraMatrix.at<double>(0, 1) = -0.283232929346427;
	// //cx
    // UpcameraMatrix.at<double>(0, 2) = 3.342769675940627e+02;
	// //fy
    // UpcameraMatrix.at<double>(1, 1) = 2.130064028240866e+02;
	// //cy
    // UpcameraMatrix.at<double>(1, 2) = 2.411115554843329e+02;
 
    // Mat UpdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
    // UpdistCoeffs.at<double>(0, 0) = -0.260185863941959; //max
    // UpdistCoeffs.at<double>(1, 0) = 0.052688535755694; // min
	// //calibrationSession.CameraParameters.TangentialDistortion  
    // UpdistCoeffs.at<double>(2, 0) = 0.002789813815027; // max
    // UpdistCoeffs.at<double>(3, 0) = -9.971230810947045e-04; // min
    // UpdistCoeffs.at<double>(4, 0) = 0;

//*********************************************************************************

	// Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
    // WatercameraMatrix.at<double>(0, 0) = 2.836571597121089e+02;
    // WatercameraMatrix.at<double>(0, 1) = 0.302994515520648;
    // WatercameraMatrix.at<double>(0, 2) = 3.308421290613233e+02;
    // WatercameraMatrix.at<double>(1, 1) = 2.647412756017969e+02;
    // WatercameraMatrix.at<double>(1, 2) = 7.558505317851999;
 
    // Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
    // WaterdistCoeffs.at<double>(0, 0) = -0.246899590875519; //max
    // WaterdistCoeffs.at<double>(1, 0) = 0.046690851958435; // min
	// //calibrationSession.CameraParameters.TangentialDistortion  
    // WaterdistCoeffs.at<double>(2, 0) = -0.001789654977026; // max
    // WaterdistCoeffs.at<double>(3, 0) = 1.795872231893150e-04; // min
    // WaterdistCoeffs.at<double>(4, 0) = 0;


	Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	//calibrationSession.CameraParameters.IntrinsicMatrix
    WatercameraMatrix.at<double>(0, 0) = 2.783149850391360e+02;
    WatercameraMatrix.at<double>(0, 1) = 0.298217438206081;
    WatercameraMatrix.at<double>(0, 2) = 3.439983226605749e+02;
    WatercameraMatrix.at<double>(1, 1) = 2.614159156300671e+02;
    WatercameraMatrix.at<double>(1, 2) = 1.180070804488273e+02;
 
    Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	//calibrationSession.CameraParameters.RadialDistortion
    WaterdistCoeffs.at<double>(0, 0) = -0.276180770814349; //max
    WaterdistCoeffs.at<double>(1, 0) = 0.063194594607371; // min
	//calibrationSession.CameraParameters.TangentialDistortion  
    WaterdistCoeffs.at<double>(2, 0) = 0.001224978291421; // max
    WaterdistCoeffs.at<double>(3, 0) = 5.661827248780201e-04; // min
    WaterdistCoeffs.at<double>(4, 0) = 0;

    Mat view, rview, map1, map2;
    Size imageSize;
    imageSize = img.size();
	Size OutImageSize(300, 300);
    initUndistortRectifyMap(UpcameraMatrix, UpdistCoeffs, Mat(),
    getOptimalNewCameraMatrix(UpcameraMatrix, UpdistCoeffs, imageSize, 0, imageSize, &ROIRect),
    imageSize, CV_16SC2, map1, map2);

	

	// initUndistortRectifyMap(WatercameraMatrix, WaterdistCoeffs, Mat(),
    // getOptimalNewCameraMatrix(WatercameraMatrix, WaterdistCoeffs, imageSize, 1, imageSize, &ROIRect),
    // imageSize, CV_16SC2, map1, map2);

	
	// fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
    //         getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0), 
	// 		image_size, CV_32FC1, mapx, mapy);

	remap(img, Calibration_Img, map1, map2, INTER_LINEAR);
} 


int stitch(Mat &a_up, Mat &b_up, Mat &dst_up, Mat &outimg, int num)
{
	// Mat a = imread("1r.png", 1);//右图  
	// Mat b = imread("1l.png", 1);//左图

	// Mat a_up = imread("1r1.jpg", 1);//右图  
	// Mat b_up = imread("1l1.jpg", 1);//左图

	// Mat a, b, temp , temp2;

	// resize(a_up, temp2, Size(640,480));

	// imshow("a", temp2);
	// imwrite("1r1a.jpg", temp2);

	// resize(b_up, temp2, Size(640,480));
	
	// imshow("b", temp2);
	// imwrite("1l1b.jpg", temp2);


	// Mat a_up = imread("1r.jpg", 1);//右图  
	// Mat b_up = imread("1l.jpg", 1);//左图

	// Mat a, b, dst_up, temp;

	// V4L2Capture_Calibration(a_up,a_up);
	// V4L2Capture_Calibration(b_up,b_up);

	// transpose(a_up, temp);
	// flip(temp,a,1);
	// imwrite("1ra.jpg", a);

	// transpose(b_up, temp);
	// flip(temp,b,1);
	// imwrite("1lb.jpg", b);

	// ****************

	struct  timeval  start;
	struct  timeval  end;
	unsigned long timer;
	gettimeofday(&start,NULL);

	Mat Pic_CutWater,temp, a, b;

	// Mat a_up = imread("1r.jpg", 1);//右图  
	// Mat b_up = imread("1l.jpg", 1);//左图

	
	Rect ROIRect(0, 0, 100, 300);

	

		
	
	// V4L2Capture_Calibration2(a_up, temp, ROIRect);
	// imwrite("1raO.jpg", temp);
	V4L2Capture_Calibration2(a_up, a_up, ROIRect);
	
	transpose(a_up, temp);
	flip(temp,a,1);//顺时针旋转
	imwrite("1ra.jpg", a);
	imwrite("1ra1.jpg", a_up(ROIRect));

	V4L2Capture_Calibration2(b_up, b_up, ROIRect);
	transpose(b_up, temp);
	flip(temp,b,1);
	imwrite("1lb.jpg", b);
	imwrite("1lb1.jpg", b_up(ROIRect));

	Rect WaterRect(0, 0, b.cols/2, b.rows); 
	
	b(WaterRect).copyTo(Pic_CutWater);
	b(WaterRect).setTo(0);
	// 	cvtColor(a, a, CV_RGB2GRAY);
    // cvtColor(b, b, CV_RGB2GRAY);

	//*************
	// Mat a_up = imread("1ra1.jpg", 1);//右图  
	// Mat b_up = imread("1lb1.jpg", 1);//左图

	
	// Rect ROIRect(0, 0, 100, 300);
	
	// transpose(a_up, temp);
	// flip(temp,a,1);//逆时针旋转
	// imwrite("1ra.jpg", a);

	// transpose(b_up, temp);
	// flip(temp,b,1);
	// imwrite("1lb.jpg", b);

	

	
	Ptr<SiftFeatureDetector> surf;            //创建方式和OpenCV2中的不一样,并且要加上命名空间xfreatures2d
							   //否则即使配置好了还是显示SURF为未声明的标识符  
	surf = SiftFeatureDetector::create(800); // 海塞矩阵阈值，在这里调整精度，值越大点越少，越精准 
 
	BFMatcher matcher;         //实例化一个暴力匹配器
	Mat c, d;
	vector<KeyPoint>key1, key2;
	vector<DMatch> matches;    //DMatch是用来描述匹配好的一对特征点的类，包含这两个点之间的相关信息
							   //比如左图有个特征m，它和右图的特征点n最匹配，这个DMatch就记录它俩最匹配，并且还记录m和n的
							   //特征向量的距离和其他信息，这个距离在后面用来做筛选
	
	surf->detectAndCompute(a, Mat(), key1, c);//输入图像，输入掩码，输入特征点，输出Mat，存放所有特征点的描述向量
	surf->detectAndCompute(b, Mat(), key2, d);//这个Mat行数为特征点的个数，列数为每个特征向量的尺寸，SURF是64（维）
	
	// writeMatToXML(c,"./c.xml");
	// writeMatToXML(d,"./d.xml");

	matcher.match(d, c, matches);             //匹配，数据来源是特征向量，结果存放在DMatch类型里面  
	// vector<vector<DMatch> > matchePoints;
	// vector<DMatch> GoodMatchePoints;
	// Mat imageDesc1, imageDesc2;

	// imageDesc1 = c;
	// imageDesc2 = d;

	// vector<Mat> train_desc(1, imageDesc1);
	// matcher.add(train_desc);
	// matcher.train();

	// matcher.knnMatch(imageDesc2, matchePoints, 2);

	// cout << "total match points: " << matchePoints.size() << endl;

	// // Lowe's algorithm,获取优秀匹配点
	// for (int i = 0; i < matchePoints.size(); i++)
	// {
	// 	if (matchePoints[i][0].distance < 0.4 * matchePoints[i][1].distance)
	// 	{
	// 		GoodMatchePoints.push_back(matchePoints[i][0]);
	// 	}
	// }

	//sort函数对数据进行升序排列
	sort(matches.begin(), matches.end());     //筛选匹配点，根据match里面特征对的距离从小到大排序
	vector< DMatch > good_matches;
	// good_matches = GoodMatchePoints;
	int ptsPairs = std::min(200, (int)(matches.size() ));
	cout << "ptsPairs:" << ptsPairs << "matches.size" << (int)matches.size() << endl;
	
	
	for (int i = 0; i < ptsPairs; i++)
	{
		good_matches.push_back(matches[i]);//距离最小的50个压入新的DMatch
		// cout << i << " queryIdx " << matches[i].queryIdx << " trainIdx " << matches[i].trainIdx << " imgIdx " << matches[i].imgIdx << " distance "<<matches[i].distance << endl;
	}


	// Mat outimg;                                //drawMatches这个函数直接画出摆在一起的图
	drawMatches(b, key2, a, key1, good_matches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  
	// drawMatches(b, key2, a, key1, GoodMatchePoints, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  

	// imshow("first_match ", outimg);
	transpose(outimg, temp);
	flip(temp,outimg,0);

	imwrite("first_match.jpg", outimg);
	
	gettimeofday(&end,NULL);
	timer = 1000000 * (end.tv_sec-start.tv_sec)+ end.tv_usec-start.tv_usec;
	cout << timer << "us" << endl;
	
	///图像配准及融合
 
	vector<Point2f> imagePoints1, imagePoints2;
 
	for (int i = 0; i<good_matches.size(); i++)
	{
		imagePoints2.push_back(key2[good_matches[i].queryIdx].pt);
		imagePoints1.push_back(key1[good_matches[i].trainIdx].pt);
	}
 
	//获取图像1到图像2的投影映射矩阵 尺寸为3*3   
	//变换1r
	Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
	///Mat homo = findHomography(imagePoints1, imagePoints2, CV_16SC);
	//也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差  
	//Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);  
	// cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵   
 
	//计算配准图的四个顶点坐标
	CalcCorners(homo, a);
	// cout << "left_top:" << corners.left_top << endl;
	// cout << "left_bottom:" << corners.left_bottom << endl;
	// cout << "right_top:" << corners.right_top << endl;
	// cout << "right_bottom:" << corners.right_bottom << endl;
 
												//图像配准  
	Mat imageTransform1, imageTransform2;
	warpPerspective(a, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), b.rows));
	//warpPerspective(a, imageTransform2, adjustMat*homo, Size(b.cols*1.3, b.rows*1.8));
	// imshow("直接经过透视矩阵变换", imageTransform1);
	imwrite("trans1.jpg", imageTransform1);
	// imageTransform1 = imread("trans1.jpg", 1);

	int dst_width;
	//创建拼接后的图,需提前计算图的大小
	if (b.cols > imageTransform1.cols)
	{
		dst_width = b.cols;
		cout << "error: " << imageTransform1.cols << endl;
	}
	else
	{
		dst_width = imageTransform1.cols;
	}
	// int dst_width = MAX(b.cols, imageTransform1.cols);  //取最右点的长度为拼接图的长度
	int dst_height = b.rows;
	
	Mat dst(dst_height, dst_width, CV_8UC3);
	dst.setTo(0);

	imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));

	Pic_CutWater.copyTo( b (WaterRect) );

	b.copyTo(dst(Rect(0, 0, b.cols, b.rows)));

	// imshow("b_dst", dst);
 

	OptimizeSeam(b, imageTransform1, dst);


	transpose(dst, temp);
	flip(temp,dst_up,0);

	// imshow("dst", dst);
	// imwrite("dst.jpg", dst);

	// imshow("dst_up", dst_up);
	imwrite("dst_up.jpg", dst_up);
	
	// waitKey();
	
	cout<<"stitch"<<endl;

}

int stitch2(Mat &a_up, Mat &b_up, Mat &dst_up, Mat &outimg, int num)
{
	// Mat a = imread("1r.png", 1);//右图  
	// Mat b = imread("1l.png", 1);//左图

	// Mat a_up = imread("1r1.jpg", 1);//右图  
	// Mat b_up = imread("1l1.jpg", 1);//左图

	// Mat a, b, temp , temp2;

	// resize(a_up, temp2, Size(640,480));

	// imshow("a", temp2);
	// imwrite("1r1a.jpg", temp2);

	// resize(b_up, temp2, Size(640,480));
	
	// imshow("b", temp2);
	// imwrite("1l1b.jpg", temp2);


	// Mat a_up = imread("1r.jpg", 1);//右图  
	// Mat b_up = imread("1l.jpg", 1);//左图

	// Mat a, b, dst_up, temp;

	// V4L2Capture_Calibration(a_up,a_up);
	// V4L2Capture_Calibration(b_up,b_up);

	// transpose(a_up, temp);
	// flip(temp,a,1);
	// imwrite("1ra.jpg", a);

	// transpose(b_up, temp);
	// flip(temp,b,1);
	// imwrite("1lb.jpg", b);

	// ****************
	Mat temp, a, b;

	// Mat a_up = imread("1r.jpg", 1);//右图  
	// Mat b_up = imread("1l.jpg", 1);//左图

	
	Rect ROIRect(0, 0, 100, 300);

	
	// V4L2Capture_Calibration2(a_up, temp, ROIRect);
	// imwrite("2raO.jpg", temp);
	V4L2Capture_Calibration2(a_up, a_up, ROIRect);
	
	transpose(a_up, temp);
	flip(temp,a,0);//逆时针旋转
	// imwrite("21ra.jpg", a);
	// imwrite("21ra1.jpg", a_up(ROIRect) );

	V4L2Capture_Calibration2(b_up, b_up, ROIRect);
	transpose(b_up, temp);
	flip(temp,b,0);
	// imwrite("21lb.jpg", b);
	// imwrite("21lb1.jpg", b_up(ROIRect) );
	// 	cvtColor(a, a, CV_RGB2GRAY);
    // cvtColor(b, b, CV_RGB2GRAY);

	//*************
	// Mat a_up = imread("1ra1.jpg", 1);//右图  
	// Mat b_up = imread("1lb1.jpg", 1);//左图

	
	// Rect ROIRect(0, 0, 100, 300);
	
	// transpose(a_up, temp);
	// flip(temp,a,1);//逆时针旋转
	// imwrite("1ra.jpg", a);

	// transpose(b_up, temp);
	// flip(temp,b,1);
	// imwrite("1lb.jpg", b);

	
	Ptr<SiftFeatureDetector> surf;            //创建方式和OpenCV2中的不一样,并且要加上命名空间xfreatures2d
							   //否则即使配置好了还是显示SURF为未声明的标识符  
	surf = SiftFeatureDetector::create(800); // 海塞矩阵阈值，在这里调整精度，值越大点越少，越精准 
 
	BFMatcher matcher;         //实例化一个暴力匹配器
	Mat c, d;
	vector<KeyPoint>key1, key2;
	vector<DMatch> matches;    //DMatch是用来描述匹配好的一对特征点的类，包含这两个点之间的相关信息
							   //比如左图有个特征m，它和右图的特征点n最匹配，这个DMatch就记录它俩最匹配，并且还记录m和n的
							   //特征向量的距离和其他信息，这个距离在后面用来做筛选
	
	surf->detectAndCompute(a, Mat(), key1, c);//输入图像，输入掩码，输入特征点，输出Mat，存放所有特征点的描述向量
	surf->detectAndCompute(b, Mat(), key2, d);//这个Mat行数为特征点的个数，列数为每个特征向量的尺寸，SURF是64（维）
	
	// writeMatToXML(c,"./c.xml");
	// writeMatToXML(d,"./d.xml");

	matcher.match(d, c, matches);             //匹配，数据来源是特征向量，结果存放在DMatch类型里面  
	// vector<vector<DMatch> > matchePoints;
	// vector<DMatch> GoodMatchePoints;
	// Mat imageDesc1, imageDesc2;

	// imageDesc1 = c;
	// imageDesc2 = d;

	// vector<Mat> train_desc(1, imageDesc1);
	// matcher.add(train_desc);
	// matcher.train();

	// matcher.knnMatch(imageDesc2, matchePoints, 2);

	// cout << "total match points: " << matchePoints.size() << endl;

	// // Lowe's algorithm,获取优秀匹配点
	// for (int i = 0; i < matchePoints.size(); i++)
	// {
	// 	if (matchePoints[i][0].distance < 0.4 * matchePoints[i][1].distance)
	// 	{
	// 		GoodMatchePoints.push_back(matchePoints[i][0]);
	// 	}
	// }

	//sort函数对数据进行升序排列
	sort(matches.begin(), matches.end());     //筛选匹配点，根据match里面特征对的距离从小到大排序
	vector< DMatch > good_matches;
	// good_matches = GoodMatchePoints;
	int ptsPairs = std::min(num, (int)(matches.size() ));
	cout << "2ptsPairs:" << ptsPairs << "matches.size" << (int)matches.size() << endl;
	
	
	for (int i = 0; i < ptsPairs; i++)
	{
		good_matches.push_back(matches[i]);//距离最小的50个压入新的DMatch
		// cout << i << " queryIdx " << matches[i].queryIdx << " trainIdx " << matches[i].trainIdx << " imgIdx " << matches[i].imgIdx << " distance "<<matches[i].distance << endl;
	}


	// Mat outimg;                                //drawMatches这个函数直接画出摆在一起的图
	drawMatches(b, key2, a, key1, good_matches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  
	// drawMatches(b, key2, a, key1, GoodMatchePoints, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  

	// imshow("2first_match ", outimg);
	imwrite("2first_match.jpg", outimg);
	transpose(outimg, temp);
	flip(temp,outimg,1);

	
	///图像配准及融合
 
	vector<Point2f> imagePoints1, imagePoints2;
 
	for (int i = 0; i<good_matches.size(); i++)
	{
		imagePoints2.push_back(key2[good_matches[i].queryIdx].pt);
		imagePoints1.push_back(key1[good_matches[i].trainIdx].pt);
	}
 
	//获取图像1到图像2的投影映射矩阵 尺寸为3*3   
	//变换1r
	Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
	///Mat homo = findHomography(imagePoints1, imagePoints2, CV_16SC);
	//也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差  
	//Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);  
	// cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵   
 
												//计算配准图的四个顶点坐标
	CalcCorners(homo, a);
	// cout << "left_top:" << corners.left_top << endl;
	// cout << "left_bottom:" << corners.left_bottom << endl;
	// cout << "right_top:" << corners.right_top << endl;
	// cout << "right_bottom:" << corners.right_bottom << endl;
 
	//图像配准  
	Mat imageTransform1, imageTransform2;
	warpPerspective(a, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), b.rows));
	//warpPerspective(a, imageTransform2, adjustMat*homo, Size(b.cols*1.3, b.rows*1.8));
	// imshow("直接经过透视矩阵变换", imageTransform1);
	imwrite("2trans1.jpg", imageTransform1);
	
	
	int dst_width;
	//创建拼接后的图,需提前计算图的大小
	if (b.cols > imageTransform1.cols)
	{
		dst_width = b.cols;
		cout << "2error: " << imageTransform1.cols << endl;
	}
	else
	{
		dst_width = imageTransform1.cols;
	}	
	//创建拼接后的图,需提前计算图的大小
	// int dst_width = MAX(b.cols, imageTransform1.cols);  //取最右点的长度为拼接图的长度
	int dst_height = b.rows;

	
	Mat dst(dst_height, dst_width, CV_8UC3);
	
	dst.setTo(0);
 
	imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	
	b.copyTo(dst(Rect(0, 0, b.cols, b.rows)));
 
	// imshow("b_dst", dst);
 
	// if (dst.rows > IMAGEHEIGHT+3)
	// {
		
	// }

	OptimizeSeam(b, imageTransform1, dst);
	transpose(dst, temp);
	flip(temp,dst_up,1);
	// imshow("dst", dst);
	// imwrite("dst.jpg", dst);

	// imshow("2dst_up", dst_up);
	imwrite("dst_up.jpg", dst_up);
	
	cout << "stitch2" << endl;
	// waitKey();
 

}


int main(int argc, char *argv[])
{
	Mat Pic_Temp, Pic_final, Pic_match1, Pic_match2;;
	// while(1)
	// {
	

		Mat Pic_up = imread("1.jpg", 1);//右图  
		Mat Pic_Mid = imread("2.jpg", 1);//左图

		stitch(Pic_up, Pic_Mid, Pic_Temp, Pic_match1, atoi(argv[1]) );
		cout<< "fi" << endl;
		imwrite("Pic_Temp.jpg", Pic_Temp);
	
		Mat Pic_Down = imread("3.jpg", 1);//左图

		Mat Pic_Temp2;
        if ( Pic_Temp.rows > IMAGEHEIGHT)
        {
            Pic_Temp( Rect (0, Pic_Temp.rows - IMAGEHEIGHT, IMAGEWIDTH, IMAGEHEIGHT) ).copyTo(Pic_Temp2);
            imwrite("Pic_Temp2.jpg", Pic_Temp2);

            stitch2(Pic_Down, Pic_Temp2, Pic_final, Pic_match2, atoi(argv[2]) );
        }
        else
        {
            stitch2(Pic_Down, Pic_Temp, Pic_final, Pic_match2, atoi(argv[2]) );
        }

		imwrite("Pic_final.jpg", Pic_final);		
	// }
	

	return 0;

}

//优化两图的连接处，使得拼接自然
void OptimizeSeam(Mat &img1, Mat &trans, Mat &dst)
{
	int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界  
	cout << "start" << start << endl;
	if (start < 0)
	{
		start = 0;
	}
	double processWidth = img1.cols - start;//重叠区域的宽度  
	int rows = dst.rows;
	int cols = img1.cols; //注意，是列数*通道数
	double alpha = 1;//img1中像素的权重  
	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			//如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1;
			}
			else
			{
				//img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
				alpha = (processWidth - (j - start)) / processWidth;
			}
 
			d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
			d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
			d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
 
		}
	}
	

}



// int main()
// {
// 	// Mat a = imread("1r.png", 1);//右图  
// 	// Mat b = imread("1l.png", 1);//左图

// 	// Mat a_up = imread("1r1.jpg", 1);//右图  
// 	// Mat b_up = imread("1l1.jpg", 1);//左图

// 	// Mat a, b, temp , temp2;

// 	// resize(a_up, temp2, Size(640,480));

// 	// imshow("a", temp2);
// 	// imwrite("1r1a.jpg", temp2);

// 	// resize(b_up, temp2, Size(640,480));
	
// 	// imshow("b", temp2);
// 	// imwrite("1l1b.jpg", temp2);


// 	// Mat a_up = imread("1r.jpg", 1);//右图  
// 	// Mat b_up = imread("1l.jpg", 1);//左图

// 	// Mat a, b, dst_up, temp;

// 	// V4L2Capture_Calibration(a_up,a_up);
// 	// V4L2Capture_Calibration(b_up,b_up);

// 	// transpose(a_up, temp);
// 	// flip(temp,a,1);
// 	// imwrite("1ra.jpg", a);

// 	// transpose(b_up, temp);
// 	// flip(temp,b,1);
// 	// imwrite("1lb.jpg", b);

// 	// ****************
// 	Mat a, b, dst_up, temp;

// 	Mat a_up = imread("1r.jpg", 1);//右图  
// 	Mat b_up = imread("1l.jpg", 1);//左图

	
// 	Rect ROIRect(0, 0, 100, 300);

	
// 	V4L2Capture_Calibration2(a_up, temp, ROIRect);
// 	imwrite("1raO.jpg", temp);
// 	V4L2Capture_Calibration(a_up, a_up, ROIRect);
	
// 	transpose(a_up, temp);
// 	flip(temp,a,1);//逆时针旋转
// 	imwrite("1ra.jpg", a);
// 	imwrite("1ra1.jpg", a_up(ROIRect));

// 	V4L2Capture_Calibration(b_up, b_up, ROIRect);
// 	transpose(b_up, temp);
// 	flip(temp,b,1);
// 	imwrite("1lb.jpg", b);
// 	imwrite("1lb1.jpg", b_up(ROIRect));
// 	// 	cvtColor(a, a, CV_RGB2GRAY);
//     // cvtColor(b, b, CV_RGB2GRAY);

// 	//*************
// 	// Mat a_up = imread("1ra1.jpg", 1);//右图  
// 	// Mat b_up = imread("1lb1.jpg", 1);//左图

	
// 	// Rect ROIRect(0, 0, 100, 300);
	
// 	// transpose(a_up, temp);
// 	// flip(temp,a,1);//逆时针旋转
// 	// imwrite("1ra.jpg", a);

// 	// transpose(b_up, temp);
// 	// flip(temp,b,1);
// 	// imwrite("1lb.jpg", b);

	

	
// 	Ptr<SiftFeatureDetector> surf;            //创建方式和OpenCV2中的不一样,并且要加上命名空间xfreatures2d
// 							   //否则即使配置好了还是显示SURF为未声明的标识符  
// 	surf = SiftFeatureDetector::create(800); // 海塞矩阵阈值，在这里调整精度，值越大点越少，越精准 
 
// 	BFMatcher matcher;         //实例化一个暴力匹配器
// 	Mat c, d;
// 	vector<KeyPoint>key1, key2;
// 	vector<DMatch> matches;    //DMatch是用来描述匹配好的一对特征点的类，包含这两个点之间的相关信息
// 							   //比如左图有个特征m，它和右图的特征点n最匹配，这个DMatch就记录它俩最匹配，并且还记录m和n的
// 							   //特征向量的距离和其他信息，这个距离在后面用来做筛选
	
// 	surf->detectAndCompute(a, Mat(), key1, c);//输入图像，输入掩码，输入特征点，输出Mat，存放所有特征点的描述向量
// 	surf->detectAndCompute(b, Mat(), key2, d);//这个Mat行数为特征点的个数，列数为每个特征向量的尺寸，SURF是64（维）
	
// 	// writeMatToXML(c,"./c.xml");
// 	// writeMatToXML(d,"./d.xml");

// 	// matcher.match(d, c, matches);             //匹配，数据来源是特征向量，结果存放在DMatch类型里面  
// 	vector<vector<DMatch> > matchePoints;
// 	vector<DMatch> GoodMatchePoints;
// 	Mat imageDesc1, imageDesc2;

// 	imageDesc1 = c;
// 	imageDesc2 = d;

// 	vector<Mat> train_desc(1, imageDesc1);
// 	matcher.add(train_desc);
// 	matcher.train();

// 	matcher.knnMatch(imageDesc2, matchePoints, 2);

// 	cout << "total match points: " << matchePoints.size() << endl;

// 	// Lowe's algorithm,获取优秀匹配点
// 	for (int i = 0; i < matchePoints.size(); i++)
// 	{
// 		if (matchePoints[i][0].distance < 0.6 * matchePoints[i][1].distance)
// 		{
// 			GoodMatchePoints.push_back(matchePoints[i][0]);
// 		}
// 	}

// 	// //sort函数对数据进行升序排列
// 	// sort(matches.begin(), matches.end());     //筛选匹配点，根据match里面特征对的距离从小到大排序
// 	vector< DMatch > good_matches;
// 	good_matches = GoodMatchePoints;
// 	// int ptsPairs = std::min(200, (int)(matches.size() * 0.15));
// 	// cout << ptsPairs << endl;
	
	
// 	// for (int i = 0; i < ptsPairs; i++)
// 	// {
// 	// 	good_matches.push_back(matches[i]);//距离最小的50个压入新的DMatch
// 	// }


// 	Mat outimg;                                //drawMatches这个函数直接画出摆在一起的图
// 	// drawMatches(b, key2, a, key1, good_matches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  
// 	drawMatches(b, key2, a, key1, GoodMatchePoints, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  

// 	imshow("first_match ", outimg);
// 	imwrite("first_match.jpg", outimg);
	

	
// 	///图像配准及融合
 
// 	vector<Point2f> imagePoints1, imagePoints2;
 
// 	for (int i = 0; i<good_matches.size(); i++)
// 	{
// 		imagePoints2.push_back(key2[good_matches[i].queryIdx].pt);
// 		imagePoints1.push_back(key1[good_matches[i].trainIdx].pt);
// 	}
 
// 	//获取图像1到图像2的投影映射矩阵 尺寸为3*3   
// 	//变换1r
// 	Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
// 	///Mat homo = findHomography(imagePoints1, imagePoints2, CV_16SC);
// 	//也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差  
// 	//Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);  
// 	cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵   
 
// 												//计算配准图的四个顶点坐标
// 	CalcCorners(homo, a);
// 	cout << "left_top:" << corners.left_top << endl;
// 	cout << "left_bottom:" << corners.left_bottom << endl;
// 	cout << "right_top:" << corners.right_top << endl;
// 	cout << "right_bottom:" << corners.right_bottom << endl;
 
// 												//图像配准  
// 	Mat imageTransform1, imageTransform2;
// 	warpPerspective(a, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), b.rows));
// 	//warpPerspective(a, imageTransform2, adjustMat*homo, Size(b.cols*1.3, b.rows*1.8));
// 	imshow("直接经过透视矩阵变换", imageTransform1);
// 	imwrite("trans1.jpg", imageTransform1);
 
// 	//创建拼接后的图,需提前计算图的大小
// 	int dst_width = imageTransform1.cols;  //取最右点的长度为拼接图的长度
// 	int dst_height = b.rows;
 
// 	Mat dst(dst_height, dst_width, CV_8UC3);
// 	dst.setTo(0);
 
// 	imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
// 	b.copyTo(dst(Rect(0, 0, b.cols, b.rows)));
 
// 	imshow("b_dst", dst);
 
 
// 	OptimizeSeam(b, imageTransform1, dst);
 
// 	transpose(dst, temp);
// 	flip(temp,dst_up,0);

// 	// imshow("dst", dst);
// 	// imwrite("dst.jpg", dst);

// 	imshow("dst_up", dst_up);
// 	imwrite("dst_up.jpg", dst_up);
	
	
// 	// waitKey();
 
// 	return 0;
// }
