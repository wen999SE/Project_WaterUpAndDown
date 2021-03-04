#include "mainwindow.h"
#include "ui_mainwindow.h"

V4L2Capture UpCap10((char*)"/dev/video10", IMAGEWIDTH, IMAGEHEIGHT);
V4L2Capture MidCap12((char*)"/dev/video12", IMAGEWIDTH, IMAGEHEIGHT);
V4L2Capture WaterCap14((char*)"/dev/video14", IMAGEWIDTH, IMAGEHEIGHT);


pthread_t Pt_Up, Pt_Mid ,Pt_MidDown ,Pt_Water;
Mat UpFrame, WaterFrame, MidFrame, MidFrame_C; 
Mat MidFrameUp(IMAGEHEIGHT/2, IMAGEWIDTH, CV_8UC3);
Mat MidFrameWater(IMAGEHEIGHT/2, IMAGEWIDTH, CV_8UC3);
Mat DownAndWaterDownFrame;
Mat MergeFramet;


vector<KeyPoint> MidFrameUp_key, MidFrameDown_key;
Mat MidFrameUp_Points, MidFrameDown_Points;


//定义无名信号量
sem_t UpFrameStart, MidFrameStart, WaterFrameStart;
sem_t MidFrameUpFinish, MidFrameWaterFinish, DownAndWaterDownFinish;
sem_t MidFrameDivideFinish;


four_corners_t corners;

struct  timeval  Time_start;
struct  timeval  Time_end;
unsigned long timer;

string imgname, string_now;


ChangeMode_Type ChangeMode = Mode_3Video_Play;//切换模式

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

//    connect(this,SIGNAL(captured_image() ),this,SLOT(Video3_Play() ) );
    connect(this,SIGNAL(MergeFinish() ),this,SLOT(Splice_Play() ) );
    
     timer_Mode1 = new QTimer(this);
     timer_Mode2 = new QTimer(this);
     connect(timer_Mode1, SIGNAL(timeout()), this, SLOT(Video3_Play()));//video play when timeout
    // connect(timer_Mode2, SIGNAL(timeout()), this, SLOT(Splice_Play()));//video play when timeout

    ui->lineEdit_SurfNum_1->setText("100");
    ui->lineEdit_SurfNum_2->setText("100");
    //创建视频流处理线程
    
    pthread_create(&Pt_Up, NULL, Pt_Up_Thread, this);
    pthread_create(&Pt_Mid, NULL, Pt_Mid_Thread, this);
    pthread_create(&Pt_MidDown, NULL, Pt_MidDown_Thread, this);
    pthread_create(&Pt_Water, NULL, Pt_Water_Thread, this);
    

    //初始化信号量

    sem_init(&UpFrameStart, 0/*作用范围是线程间*/, 1/*初始值*/);
    sem_init(&MidFrameStart, 0/*作用范围是线程间*/, 1/*初始值*/);
    sem_init(&WaterFrameStart, 0/*作用范围是线程间*/, 1/*初始值*/);

    sem_init(&MidFrameUpFinish, 0/*作用范围是线程间*/, 0/*初始值*/);
    sem_init(&MidFrameWaterFinish, 0/*作用范围是线程间*/, 0/*初始值*/);
    
    sem_init(&DownAndWaterDownFinish, 0/*作用范围是线程间*/, 0/*初始值*/);
    
    sem_init(&MidFrameDivideFinish, 0/*作用范围是线程间*/, 0/*初始值*/);

    timer_Mode1->start(33);
}

/**
  * @brief(简述) 	图片线程处理
  * @note(注意)   	This function.
  * @param(入参)  	None
  * @retval(返回值) None
  */

void *Pt_Mid_Thread(void* args)
{
    MainWindow *ptr_MainWindow = (MainWindow *)args;
    ptr_MainWindow->Pt_Mid_ThreadCallBack();
    return NULL;
}

void *Pt_Water_Thread(void* args)
{
    MainWindow *ptr_MainWindow = (MainWindow *)args;
    ptr_MainWindow->Pt_Water_ThreadCallBack();
    return NULL;
}
void *Pt_MidDown_Thread(void* args)
{
    MainWindow *ptr_MainWindow = (MainWindow *)args;
    ptr_MainWindow->Pt_MidDown_ThreadCallBack();
    return NULL;
}


void MainWindow::Pt_Up_ThreadCallBack(void)
{
    Mat UpFrame_C, temp, Pic_CutWater;
    

    Mat imageDesc1, imageDesc2;

    Rect ROIRect(0, 0, 100, 300);

    Ptr<SURF> surf;			  //创建方式和OpenCV2中的不一样,并且要加上命名空间xfreatures2d
                                    //否则即使配置好了还是显示SURF为未声明的标识符
    surf = SURF::create(800); // 海塞矩阵阈值，在这里调整精度，值越大点越少，越精准

    BFMatcher matcher; //实例化一个暴力匹配器
    Mat c, d;
 

    int num;

    int i;
    Mat homo ,imageTransform1, imageTransform2;
    int UpAndWaterUpFrame_width; 
    
    while(1)
    {
        UpCap10 >> UpFrame;           
        if (UpFrame.empty())
        {
            cout << "UpFrame empty !" << endl;
            break;  
        } 
        switch(ChangeMode)
        {
            case Mode_3Video_Play:
            break;
            

//*********************************************************************************
            
            case Mode_Splice_Play:
            
            //开始拼接指令
            sem_wait(&UpFrameStart);
            
//            V4L2Capture_Calibration_Water(UpFrame, UpFrame_C, ROIRect);
            V4L2Capture_Calibration_Up(UpFrame, UpFrame_C, ROIRect);

#ifndef     Release

            string_now = to_string( time(0) );
            mkdir("./getC/uo",S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

            imwrite("./getC/uo/"+string_now+"UpFrame.jpg", UpFrame);
            imwrite("./getC/uo/"+string_now+"UpFrame_C.jpg", UpFrame_C);
            imwrite("./getC/uo/"+string_now+"UpFrame_C1.jpg", UpFrame_C(ROIRect));

#endif         
            transpose(UpFrame_C, temp);
        	flip(temp,UpFrame_C,1);//顺时针旋转
            
//开始图片拼接*********************************************************************************                    
            num = ( ui->lineEdit_SurfNum_1->text() ).toInt();

            vector<DMatch> matches; //DMatch是用来描述匹配好的一对特征点的类，包含这两个点之间的相关信息
                            //比如左图有个特征m，它和右图的特征点n最匹配，这个DMatch就记录它俩最匹配，并且还记录m和n的
                            //特征向量的距离和其他信息，这个距离在后面用来做筛选
                            
            vector<KeyPoint> key1;
            surf->detectAndCompute(UpFrame_C, Mat(), key1, c); //输入图像，输入掩码，输入特征点，输出Mat，存放所有特征点的描述向量

            
             //这个Mat行数为特征点的个数，列数为每个特征向量的尺寸，SURF是64（维）
             
            //等待MidFrameUp图片处理完成
            sem_wait(&MidFrameUpFinish);
             
            matcher.match(MidFrameUp_Points, c, matches); //匹配，数据来源是特征向量，结果存放在DMatch类型里面

            //sort函数对数据进行升序排列
            sort(matches.begin(), matches.end()); //筛选匹配点，根据match里面特征对的距离从小到大排序
            
            int ptsPairs = std::min(num, (int)(matches.size()));
            vector<DMatch> good_matches;
            for (i = 0; i < ptsPairs; i++)
            {
                good_matches.push_back(matches[i]); //距离最小的50个压入新的DMatch
            }
            cout << "ptsPairs: " << ptsPairs << " matches.size" << (int)matches.size() << endl;
            

#ifndef     Release

            if (!(c.empty()) || !(MidFrameUp_Points.empty()) )
	        {
                cout << MidFrameUp_key.size() << "\r\n" <<key1.size() <<endl;
                Mat outimg;                                //drawMatches这个函数直接画出摆在一起的图
                drawMatches(MidFrameUp, MidFrameUp_key, UpFrame_C, key1, good_matches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS); //绘制匹配点
                imwrite("./getC/uo/"+string_now+"first_match.jpg", outimg);

                Mat ShowKeypoints1, ShowKeypoints2;
                drawKeypoints(MidFrameUp,MidFrameUp_key,ShowKeypoints1);
                drawKeypoints(UpFrame_C, key1, ShowKeypoints2);

                imwrite("./getC/uo/"+string_now+"ShowKeypoints1.jpg", ShowKeypoints1);
                imwrite("./getC/uo/"+string_now+"ShowKeypoints2.jpg", ShowKeypoints2);
            }
            else      
            {
                cout << "drawMatches error" << endl;
            }
            
#endif            
            ///图像配准及融合

            vector<Point2f> imagePoints1, imagePoints2;
            
            for (i = 0; i < (int)good_matches.size(); i++)
            {
                imagePoints2.push_back(MidFrameUp_key[good_matches[i].queryIdx].pt);
                imagePoints1.push_back(key1[good_matches[i].trainIdx].pt);
            }

            //获取图像1到图像2的投影映射矩阵 尺寸为3*3
            //变换1r
            homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
            //计算配准图的四个顶点坐标
            CalcCorners(homo, UpFrame_C);
            //图像配准
            
            warpPerspective(UpFrame_C, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), MidFrameUp.rows ) );
#ifndef     Release

            imwrite("./getC/uo/"+string_now+"trans1.jpg", imageTransform1);
#endif
            //创建拼接后的图,需提前计算图的大小
            
            if (MidFrameUp.cols > imageTransform1.cols)
            {
                UpAndWaterUpFrame_width = MidFrameUp.cols;
                cout << "error: " << imageTransform1.cols << endl;
            }
            else
            {
                UpAndWaterUpFrame_width = imageTransform1.cols;
            }

            
            int UpAndWaterUpFrame_height = MidFrameUp.rows;
            Mat UpAndWaterUpFrame(UpAndWaterUpFrame_height, UpAndWaterUpFrame_width, CV_8UC3);
            UpAndWaterUpFrame.setTo(0);

            imageTransform1.copyTo(UpAndWaterUpFrame(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
            MidFrameUp.copyTo(UpAndWaterUpFrame(Rect(0, 0, MidFrameUp.cols, MidFrameUp.rows)));
            
            
            OptimizeSeam(MidFrameUp, imageTransform1, UpAndWaterUpFrame);
            transpose(UpAndWaterUpFrame, temp);
        	flip(temp,UpAndWaterUpFrame,0);//逆时针旋转
            
//*********************************************************************************
//          等待水下拼接完成
            sem_wait(&DownAndWaterDownFinish);

        	
#ifndef     Release         
            mkdir("./getC/fi",S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

            imwrite("./getC/fi/"+string_now+"UpAndWaterUpFrame.jpg", UpAndWaterUpFrame);
            imwrite("./getC/fi/"+string_now+"DownAndWaterDownFinish.jpg", DownAndWaterDownFrame);
            
#endif
            Mat MergeFrame(DownAndWaterDownFrame.rows + UpAndWaterUpFrame.rows, DownAndWaterDownFrame.cols, CV_8UC3);   
            UpAndWaterUpFrame.copyTo( MergeFrame( Rect( 0, 0,UpAndWaterUpFrame.cols, UpAndWaterUpFrame.rows) ) );
            DownAndWaterDownFrame.copyTo( MergeFrame( Rect( 0, UpAndWaterUpFrame.rows ,DownAndWaterDownFrame.cols, DownAndWaterDownFrame.rows) ) );

#ifndef     Release
            imwrite("./getC/fi/"+string_now+"MergeFrame.jpg", MergeFrame);
#endif
            transpose(MergeFrame, temp);
            flip(temp,MergeFrame,1);//顺时针旋转

            MergeFramet = MergeFrame;
//*********************************************************************************
            emit MergeFinish();

//*********************************************************************************



            gettimeofday(&Time_end,NULL);
            timer = 1000000 * (Time_end.tv_sec-Time_start.tv_sec)+ Time_end.tv_usec-Time_start.tv_usec;
            cout << timer << "Startus" << endl << endl;
            
            //完成一次图片拼接
            sem_post(&UpFrameStart);
            sem_post(&MidFrameStart);
            sem_post(&WaterFrameStart);
            
            break;
        }     
        
//*********************************************************************************

        //释放内存
        cvReleaseImage(& (UpCap10.Capture_IplImage) ); 
    }

    pthread_exit(NULL);
}

void MainWindow::Pt_Mid_ThreadCallBack(void)
{
    Mat temp;
    Rect ROIRect(0, 0, 100, 300);
    Ptr<SURF> surf;
    surf = SURF::create(800);
    
    while(1)
    {
        MidCap12 >> MidFrame;
        if (MidFrame.empty()) 
        {
            cout << "MidFrame empty !" << endl;
            break;  
        }   

        switch(ChangeMode)
        {
            case Mode_3Video_Play:

            break;
            
            case Mode_Splice_Play:
            
//得到MidFrameUp和MidFrameWater**************************************************************
            //等待整体图片拼接完成
            sem_wait(&MidFrameStart);

            
            //清除图片信息
            MidFrameUp.setTo(0);
            MidFrameWater.setTo(0);
    
            //畸变校正
            
            V4L2Capture_Calibration_Water(MidFrame, MidFrame_C, ROIRect);
            
#ifndef     Release
            mkdir("./getC/mo",S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
            
            imwrite("./getC/mo/"+string_now+"MidFrame.jpg", MidFrame);
            imwrite("./getC/mo/"+string_now+"MidFrameC.jpg", MidFrame_C);
            imwrite("./getC/mo/"+string_now+"MidFrameC1.jpg", MidFrame_C(ROIRect));
#endif
//            MidFrameUp = MidFrame_C;
//            Rect WaterRect(0, IMAGEHEIGHT - MidFrameUp.rows / 2, MidFrameUp.cols,MidFrameUp.rows / 2);
//            //得到MidFrameWater
//            MidFrameUp(WaterRect).copyTo( MidFrameWater(WaterRect) );
//            //水下部分清除
//            MidFrameUp(WaterRect).setTo(0);

            MidFrame_C(Rect(0 ,0 , MidFrame_C.cols,MidFrame_C.rows / 2) ).copyTo( MidFrameUp );
            MidFrame_C(Rect(0 ,IMAGEHEIGHT / 2, MidFrame_C.cols,MidFrame_C.rows / 2) ).copyTo( MidFrameWater );

            transpose(MidFrameUp, temp);
        	flip(temp, MidFrameUp, 1);//顺时针旋转

            transpose(MidFrameWater, temp);
        	flip(temp, MidFrameWater, 0);//逆时针旋转
        	
            //通知MidFrameWater可以进行特征提取
            sem_post(&MidFrameDivideFinish);          
//*********************************************************************************

            surf->detectAndCompute(MidFrameUp, Mat(), MidFrameUp_key, MidFrameUp_Points);  
            
//*********************************************************************************

#ifndef     Release
            imwrite("./getC/mo/"+string_now+"MidFrameUp.jpg", MidFrameUp);
            imwrite("./getC/mo/"+string_now+"MidFrameWater.jpg", MidFrameWater);
#endif         
            //图片分割完成,通知Pt_Up
            sem_post(&MidFrameUpFinish);


//*********************************************************************************

            break;
        } 

        //释放内存
        cvReleaseImage(& (MidCap12.Capture_IplImage) );       
        
    }
    pthread_exit(NULL);
}

void MainWindow::Pt_MidDown_ThreadCallBack(void)
{
    Ptr<SURF> surf;
    surf = SURF::create(800);
    while(1)
    {
        sem_wait(&MidFrameDivideFinish);
        
        surf->detectAndCompute(MidFrameWater, Mat(), MidFrameDown_key, MidFrameDown_Points);

        sem_post(&MidFrameWaterFinish);

    }
    pthread_exit(NULL);
}


void MainWindow::Pt_Water_ThreadCallBack(void)
{

    Mat WaterFrame_C, temp, Pic_CutWaterUp;
    Rect ROIRect(0, 0, 100, 300);

    Rect WaterUpRect(0, 0, 120, IMAGEWIDTH);
    int num;
    Ptr<SURF> surf;           //创建方式和OpenCV2中的不一样,并且要加上命名空间xfreatures2d
    						//否则即使配置好了还是显示SURF为未声明的标识符
    surf = SURF::create(800); // 海塞矩阵阈值，在这里调整精度，值越大点越少，越精准

    BFMatcher matcher; //实例化一个暴力匹配器
    Mat c, d;

    Mat imageDesc1, imageDesc2;									

    int i;

    int DownAndWaterDownFrame_width; //取最右点的长度为拼接图的长度
    Mat imageTransform1, imageTransform2;
    
    while(1)
    {
        WaterCap14 >> WaterFrame;
        if (WaterFrame.empty()) 
        {
            cout << "WaterFrame empty !" << endl;
            break;  
        } 

        switch(ChangeMode)
        {   
            //单独显示每个摄像头
            case Mode_3Video_Play:


            break;

            
            //显示拼接的图像
            case Mode_Splice_Play:

            sem_wait(&WaterFrameStart);

            
            V4L2Capture_Calibration_Water(WaterFrame, WaterFrame_C, ROIRect);
            

            
#ifndef     Release
            mkdir("./getC/wo",S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

            imwrite("./getC/wo/"+string_now+"WaterFrame.jpg", WaterFrame);
            
            imwrite("./getC/wo/"+string_now+"WaterFrame_C1.jpg", WaterFrame_C(ROIRect));
#endif         

            
            transpose(WaterFrame_C, temp);
        	flip(temp, WaterFrame_C, 0);//逆时针旋转

            

        	WaterFrame_C(WaterUpRect).copyTo(Pic_CutWaterUp);
        	WaterFrame_C(WaterUpRect).setTo(0);
#ifndef     Release
            imwrite("./getC/wo/"+string_now+"WaterFrame_C.jpg", WaterFrame_C);
            imwrite("./getC/wo/"+string_now+"2Pic_CutWaterUp.jpg", Pic_CutWaterUp);
#endif

            num = ( ui->lineEdit_SurfNum_2->text() ).toInt();
//开始图片拼接*********************************************************************************                    
            
            vector<KeyPoint> key1;
            vector<DMatch> matches;
            surf->detectAndCompute(WaterFrame_C, Mat(), key1, c); //输入图像，输入掩码，输入特征点，输出Mat，存放所有特征点的描述向量


             //这个Mat行数为特征点的个数，列数为每个特征向量的尺寸，SURF是64（维）

            //等待MidFrameWater图片处理完成
            sem_wait(&MidFrameWaterFinish);
             
            matcher.match(MidFrameDown_Points, c, matches); //匹配，数据来源是特征向量，结果存放在DMatch类型里面
            
            
            
            //sort函数对数据进行升序排列
            sort(matches.begin(), matches.end()); //筛选匹配点，根据match里面特征对的距离从小到大排序
            
            int ptsPairs = std::min(num, (int)(matches.size()));
            cout << "2ptsPairs:" << ptsPairs << " 2matches.size" << (int)matches.size() << endl;
            vector<DMatch> good_matches;
            for (i = 0; i < ptsPairs; i++)
            {
                good_matches.push_back(matches[i]); //距离最小的50个压入新的DMatch
            }

            
            
#ifndef     Release
             if (!(c.empty()) || !(MidFrameDown_Points.empty()) )
	        {
                cout << MidFrameDown_key.size() << "\r\n" <<key1.size() <<endl;
                Mat outimg;                                //drawMatches这个函数直接画出摆在一起的图
                drawMatches(MidFrameWater, MidFrameDown_key, WaterFrame_C, key1, good_matches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS); //绘制匹配点
                imwrite("./getC/wo/"+string_now+"2first_match.jpg", outimg);

                Mat ShowKeypoints1, ShowKeypoints2;
                drawKeypoints(MidFrameWater,MidFrameDown_key,ShowKeypoints1);
                drawKeypoints(WaterFrame_C, key1, ShowKeypoints2);

                imwrite("./getC/wo/"+string_now+"ShowKeypoints1.jpg", ShowKeypoints1);
                imwrite("./getC/wo/"+string_now+"ShowKeypoints2.jpg", ShowKeypoints2);
            }
             else      
            {
                cout << "drawMatches error" << endl;
            }
#endif            
            ///图像配准及融合
            
            
            vector<Point2f> imagePoints1, imagePoints2;
            for (i = 0; i < (int)good_matches.size(); i++)
            {
                imagePoints2.push_back(MidFrameDown_key[good_matches[i].queryIdx].pt);
                imagePoints1.push_back(key1[good_matches[i].trainIdx].pt);
            }

            //获取图像1到图像2的投影映射矩阵 尺寸为3*3
            //变换1r
            Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
            //计算配准图的四个顶点坐标
            CalcCorners(homo, WaterFrame_C);
            //图像配准
            
            
            warpPerspective(WaterFrame_C, imageTransform1, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), MidFrameWater.rows ) );
#ifndef     Release
            imwrite("./getC/wo/"+string_now+"2trans1.jpg", imageTransform1);
#endif
            Pic_CutWaterUp.copyTo( WaterFrame_C(WaterUpRect) );

            //创建拼接后的图,需提前计算图的大小
            

            if (MidFrameWater.cols > imageTransform1.cols)
            {
                DownAndWaterDownFrame_width = MidFrameWater.cols;
                cout << "2error: " << imageTransform1.cols << endl;
            }
            else
            {
                DownAndWaterDownFrame_width = imageTransform1.cols;
            }
            
			int DownAndWaterDownFrame_height = MidFrameWater.rows;
            Mat DownAndWaterDownFrame2(DownAndWaterDownFrame_height, DownAndWaterDownFrame_width, CV_8UC3);
               
            DownAndWaterDownFrame2.setTo(0);

            imageTransform1.copyTo(DownAndWaterDownFrame2(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
            // Pic_CutWater.copyTo(MidFrameUp(WaterRect));
            MidFrameWater.copyTo(DownAndWaterDownFrame2(Rect(0, 0, MidFrameWater.cols, MidFrameWater.rows)));

            DownAndWaterDownFrame = DownAndWaterDownFrame2;
            
            OptimizeSeam(MidFrameWater, imageTransform1, DownAndWaterDownFrame);
            transpose(DownAndWaterDownFrame, temp);
        	flip(temp, DownAndWaterDownFrame, 1);//顺时针旋转

//*********************************************************************************
            //完成一次图片拼接
            sem_post(&DownAndWaterDownFinish);
            break;
        } 

        //释放内存
        cvReleaseImage(& (WaterCap14.Capture_IplImage) );  
    }
    pthread_exit(NULL);
}

//*********************************************************************************

MainWindow::~MainWindow()
{
    // 线程资源回收
    // pthread_join(Pt_Up, NULL);
    // pthread_join(Pt_Mid, NULL);
    // pthread_join(Pt_Water, NULL);
    cout<< "exit Splice!" << endl;
    delete ui;
}

void MainWindow::on_Change_pushButton_clicked()
{


    if (ChangeMode == Mode_3Video_Play) //显示图像拼接视图
    {
        timer_Mode1->stop();

        // timer_Mode1->start(33);
        //切换下一次状态
        ChangeMode = Mode_Splice_Play;

        ui->Change_pushButton->setText("SplicePlay");

        //隐藏三个显示界面
        ui->CameraUp_label->setVisible (false); 
//        ui->CameraMid_label->setVisible (false); 
        ui->CameraDown_label->setVisible (false); 
        
        ui->CameraUp_label->clear();
        ui->CameraMid_label->clear();
        ui->CameraDown_label->clear();
        
        
        //显示拼接界面
        ui->CameraSplice_label->setVisible (true);

        
        gettimeofday(&Time_start,NULL);
    }
    else //显示普通视图
    {
        timer_Mode1->start(33);
        // timer_Mode1->stop();
        //切换状态
        ChangeMode = Mode_3Video_Play;

        ui->Change_pushButton->setText("3VideoPlay");
        
        
        //显示三个显示界面
        ui->CameraUp_label->setVisible (true); 
//        ui->CameraMid_label->setVisible (true); 
        ui->CameraDown_label->setVisible (true); 
        //隐藏拼接界面
//        sleep(1);
        
        ui->CameraSplice_label->setVisible (false);
        ui->CameraSplice_label->clear();

        

        //timer_Mode1->start(33);
       /* cvtColor(srcImg, temp, CV_BGR2RGB);//BGR convert to RGB
        Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);

        ui->UpCamera_label->setPixmap(QPixmap::fromImage(Qtemp));
        ui->UpCamera_label->resize(Qtemp.size());
        ui->UpCamera_label->show();*/
    }
}


// void MainWindow::Video2_Play()
// {
    // Mat temp, frame, waterframe;
    // QImage Qtemp;

    // UpCap10 >> frame;              // read
    // WaterCap14 >> waterframe;
    // if (frame.empty()) break;         // check if at end
    // if (waterframe.empty()) break;         // check if at end

    

    // cvtColor(waterframe, temp, CV_BGR2RGB);//BGR convert to RGB
    // Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);

    // ui->CameraDown_label->setPixmap(QPixmap::fromImage(Qtemp));
    // ui->CameraDown_label->resize(Qtemp.size());
    // ui->CameraDown_label->show();

    
    // cvReleaseImage(& (WaterCap12.Capture_IplImage) );
// }


// int CameraInit()
// {

    

// }


void CalcCorners(Mat &H, const Mat &src)
{
	double v2[] = {0, 0, 1};		  //左上角
	double v1[3];					  //变换后的坐标值
	double v3[] = {1, 1, 1,1,1,1,1,1,1};
	Mat V2 = Mat(3, 1, CV_64FC1, v2); //列向量
	Mat V1 = Mat(3, 1, CV_64FC1, v1); //列向量
	if (H.empty())
	{
        H = Mat(3, 3, CV_64FC1, v3);
        
	}
    cout << H << endl;
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
	V2 = Mat(3, 1, CV_64FC1, v2); //列向量
	V1 = Mat(3, 1, CV_64FC1, v1); //列向量
	V1 = H * V2;
	corners.left_bottom.x = v1[0] / v1[2];
	corners.left_bottom.y = v1[1] / v1[2];

	//右上角(src.cols,0,1)
	v2[0] = src.cols;
	v2[1] = 0;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2); //列向量
	V1 = Mat(3, 1, CV_64FC1, v1); //列向量
	V1 = H * V2;
	corners.right_top.x = v1[0] / v1[2];
	corners.right_top.y = v1[1] / v1[2];

	//右下角(src.cols,src.rows,1)
	v2[0] = src.cols;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2); //列向量
	V1 = Mat(3, 1, CV_64FC1, v1); //列向量
	V1 = H * V2;
	corners.right_bottom.x = v1[0] / v1[2];
	corners.right_bottom.y = v1[1] / v1[2];
}

int V4L2Capture_Calibration_Water(cv::Mat &img, cv::Mat &Calibration_Img, Rect &ROIRect)
{

	// cv::Mat cameraMatrix;
	// cv::Mat distCoeffs;

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

//*********************************************************************************

	// Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
	// WatercameraMatrix.at<double>(0, 0) = 2.783149850391360e+02;
	// WatercameraMatrix.at<double>(0, 1) = 0.298217438206081;
	// WatercameraMatrix.at<double>(0, 2) = 3.439983226605749e+02;
	// WatercameraMatrix.at<double>(1, 1) = 2.614159156300671e+02;
	// WatercameraMatrix.at<double>(1, 2) = 1.180070804488273e+02;

	// Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
	// WaterdistCoeffs.at<double>(0, 0) = -0.276180770814349;	  //max
	// WaterdistCoeffs.at<double>(1, 0) = 0.063194594607371;	  // min
	// 														  //calibrationSession.CameraParameters.TangentialDistortion
	// WaterdistCoeffs.at<double>(2, 0) = 0.001224978291421;	  // max
	// WaterdistCoeffs.at<double>(3, 0) = 5.661827248780201e-04; // min
	// WaterdistCoeffs.at<double>(4, 0) = 0;

//*********************************************************************************
	//2021年1月20日23:35:44
	// Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
	// WatercameraMatrix.at<double>(0, 0) = 2.787863346904819e+02;
	// WatercameraMatrix.at<double>(0, 1) = 0.010028396098492;
	// WatercameraMatrix.at<double>(0, 2) = 3.397574601646206e+02;
	// WatercameraMatrix.at<double>(1, 1) = 2.615671305189122e+02;
	// WatercameraMatrix.at<double>(1, 2) = 2.573506471990538e+02;

	// Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
	// WaterdistCoeffs.at<double>(0, 0) = -0.292755793572702;	  //max
	// WaterdistCoeffs.at<double>(1, 0) = 0.081051209467347;	  // min
	// 														  //calibrationSession.CameraParameters.TangentialDistortion
	// WaterdistCoeffs.at<double>(2, 0) = 8.450940959506473e-04;	  // max
	// WaterdistCoeffs.at<double>(3, 0) = 6.338449853847589e-04; // min
	// WaterdistCoeffs.at<double>(4, 0) = 0;
	
//*********************************************************************************
	// 2021年1月21日10:20:11
	Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	//calibrationSession.CameraParameters.IntrinsicMatrix
	WatercameraMatrix.at<double>(0, 0) = 2.798087731660834e+02;
	WatercameraMatrix.at<double>(0, 1) = -0.736592071512236;
	WatercameraMatrix.at<double>(0, 2) = 3.416191102586665e+02;
	WatercameraMatrix.at<double>(1, 1) = 2.613262805766725e+02;
	WatercameraMatrix.at<double>(1, 2) = 2.466167828455781e+02;

	Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	//calibrationSession.CameraParameters.RadialDistortion
	WaterdistCoeffs.at<double>(0, 0) = -0.262876787653799;	  //max
	WaterdistCoeffs.at<double>(1, 0) = 0.058112544727391;	  // min
															  //calibrationSession.CameraParameters.TangentialDistortion
	WaterdistCoeffs.at<double>(2, 0) = -5.257552501089222e-04;	  // max
	WaterdistCoeffs.at<double>(3, 0) = -8.907200014895500e-04; // min
	WaterdistCoeffs.at<double>(4, 0) = 0;
	
//*********************************************************************************




	Mat view, rview, map1, map2;
	Size imageSize;
	imageSize = img.size();
	// initUndistortRectifyMap(UpcameraMatrix, UpdistCoeffs, Mat(),
	// getOptimalNewCameraMatrix(UpcameraMatrix, UpdistCoeffs, imageSize, 1, imageSize, 0),
	// imageSize, CV_16SC2, map1, map2);

	initUndistortRectifyMap(WatercameraMatrix, WaterdistCoeffs, Mat(),
							getOptimalNewCameraMatrix(WatercameraMatrix, WaterdistCoeffs, imageSize, 0, imageSize, &ROIRect),
							imageSize, CV_16SC2, map1, map2);
	// fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
	//         getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0),
	// 		image_size, CV_32FC1, mapx, mapy);

	remap(img, Calibration_Img, map1, map2, INTER_LINEAR);
    return 0;
}

int V4L2Capture_Calibration_Up(cv::Mat &img, cv::Mat &Calibration_Img, Rect &ROIRect)
{

	// cv::Mat cameraMatrix;
	// cv::Mat distCoeffs;

	//mo
	// Mat UpcameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
	// UpcameraMatrix.at<double>(0, 0) = 5.185880932630046e+02;
	// UpcameraMatrix.at<double>(0, 1) = 17.336954083655893;
	// UpcameraMatrix.at<double>(0, 2) = 3.087822303620704e+02;
	// UpcameraMatrix.at<double>(1, 1) = 4.555369092551889e+02;
	// UpcameraMatrix.at<double>(1, 2) = 3.423304057754957e+02;

	// Mat UpdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
	// UpdistCoeffs.at<double>(0, 0) = -0.529187877052290; //max
	// UpdistCoeffs.at<double>(1, 0) = 0.105261129928427;	// min
	// 													//calibrationSession.CameraParameters.TangentialDistortion
	// UpdistCoeffs.at<double>(2, 0) = -0.061167435606287; // max
	// UpdistCoeffs.at<double>(3, 0) = 0.031070402666666;	// min
	// UpdistCoeffs.at<double>(4, 0) = 0;

	//*********************************************************************************

	//2021年1月20日12:12:33
	Mat UpcameraMatrix = Mat::eye(3, 3, CV_64F);

	//calibrationSession.CameraParameters.IntrinsicMatrix
	UpcameraMatrix.at<double>(0, 0) = 2.323019204389030e+02;
	UpcameraMatrix.at<double>(0, 1) = 0.350544677720520;
	UpcameraMatrix.at<double>(0, 2) = 3.095279886623994e+02;
	UpcameraMatrix.at<double>(1, 1) = 2.172422109352863e+02;
	UpcameraMatrix.at<double>(1, 2) = 2.270702731506381e+02;

	Mat UpdistCoeffs = Mat::zeros(5, 1, CV_64F);
	//calibrationSession.CameraParameters.RadialDistortion
	UpdistCoeffs.at<double>(0, 0) = -0.301321271878391; //max
	UpdistCoeffs.at<double>(1, 0) = 0.077620602255518;	// min
	//calibrationSession.CameraParameters.TangentialDistortion
	UpdistCoeffs.at<double>(2, 0) = 9.491367930056856e-04; // max
	UpdistCoeffs.at<double>(3, 0) = 1.910594867427028e-04;	// min
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

	// Mat WatercameraMatrix = Mat::eye(3, 3, CV_64F);

	// //calibrationSession.CameraParameters.IntrinsicMatrix
	// WatercameraMatrix.at<double>(0, 0) = 2.783149850391360e+02;
	// WatercameraMatrix.at<double>(0, 1) = 0.298217438206081;
	// WatercameraMatrix.at<double>(0, 2) = 3.439983226605749e+02;
	// WatercameraMatrix.at<double>(1, 1) = 2.614159156300671e+02;
	// WatercameraMatrix.at<double>(1, 2) = 1.180070804488273e+02;

	// Mat WaterdistCoeffs = Mat::zeros(5, 1, CV_64F);
	// //calibrationSession.CameraParameters.RadialDistortion
	// WaterdistCoeffs.at<double>(0, 0) = -0.276180770814349;	  //max
	// WaterdistCoeffs.at<double>(1, 0) = 0.063194594607371;	  // min
	// 														  //calibrationSession.CameraParameters.TangentialDistortion
	// WaterdistCoeffs.at<double>(2, 0) = 0.001224978291421;	  // max
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
    return 0;
}

//优化两图的连接处，使得拼接自然
void OptimizeSeam(Mat &img1, Mat &trans, Mat &dst)
{
	int start = MIN(corners.left_top.x, corners.left_bottom.x); //开始位置，即重叠区域的左边界
	cout << "start" << start << endl;
	if (start < 0)
	{
		start = 0;
	}
	double processWidth = img1.cols - start; //重叠区域的宽度
	int rows = dst.rows;
	int cols = img1.cols; //注意，是列数*通道数
	double alpha = 1;	  //img1中像素的权重
	for (int i = 0; i < rows; i++)
	{
		uchar *p = img1.ptr<uchar>(i); //获取第i行的首地址
		uchar *t = trans.ptr<uchar>(i);
		uchar *d = dst.ptr<uchar>(i);
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

void MainWindow::Video3_Play(void)
{
    
    QImage Qtemp;
    Mat temp;
    
    cvtColor(UpFrame, temp, CV_BGR2RGB);//BGR convert to RGB
    Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);

    ui->CameraSplice_label->clear();
    ui->CameraUp_label->setPixmap(QPixmap::fromImage(Qtemp));
    ui->CameraUp_label->resize(Qtemp.size());
    ui->CameraUp_label->show();

    cvtColor(MidFrame, temp, CV_BGR2RGB);//BGR convert to RGB
    Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);

    ui->CameraMid_label->setPixmap(QPixmap::fromImage(Qtemp));
    ui->CameraMid_label->resize(Qtemp.size());
    ui->CameraMid_label->show();

    cvtColor(WaterFrame, temp, CV_BGR2RGB);//BGR convert to RGB
    Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);

    ui->CameraDown_label->setPixmap(QPixmap::fromImage(Qtemp));
    ui->CameraDown_label->resize(Qtemp.size());
    ui->CameraDown_label->show();

//    sem_post(&DisplayFrame);              
}
void MainWindow::Splice_Play(void)
{
    QImage Qtemp;
    Mat temp;
    
    cvtColor(MergeFramet, temp, CV_BGR2RGB);
    Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);

    ui->CameraSplice_label->clear();
    ui->CameraSplice_label->setPixmap(QPixmap::fromImage(Qtemp));
    ui->CameraSplice_label->resize(Qtemp.size());
    ui->CameraSplice_label->show(); 

    cvtColor(MidFrame_C, temp, CV_BGR2RGB);//BGR convert to RGB
    Qtemp = QImage((const unsigned char*)(temp.data), temp.cols, temp.rows, temp.step, QImage::Format_RGB888);

    ui->CameraMid_label->setPixmap(QPixmap::fromImage(Qtemp));
    ui->CameraMid_label->resize(Qtemp.size());
    ui->CameraMid_label->show();
}


