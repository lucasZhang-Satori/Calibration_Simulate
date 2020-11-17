//
// Created by zzl on 2020/11/11.
//

// System
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

// Eigen
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

// OpenCV
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/viz.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace std;
using namespace cv;

int main(int argc,char** argv){
    viz::Viz3d mainWindow("MainWindow"); // 创建主窗口
    viz::Viz3d camWindow("CamWindow"); // 创建相机窗口
    mainWindow.spinOnce(); // 初始化
    camWindow.spinOnce(); // 初始化
    // Init Cam
    Matx33f Intrisic(700,0,360,0,700,240,0,0,1); // 相机内参数
    viz::Camera Cam(Intrisic,Size(720,480)); // 设置相机
    viz::WCameraPosition camOrient_main(Cam.getFov(),1.0,viz::Color::blue()); // 相机位置（主窗口）
    viz::WCameraPosition camOrient_cam(Cam.getFov(),1.0,viz::Color::blue()); // 相机视角
    Affine3f camPosition(Mat::eye(3,3,CV_32F),Vec3f(0,0,-8)); // 相机窗口中的观测位姿(相机位置也就是观测位姿)

    // Observe Position
    Eigen::AngleAxisf Rotation_x(CV_PI / 2 ,Eigen::Vector3f(1,0,0)); // 主窗口中的观测位姿
    Mat RotationX;
    eigen2cv(Rotation_x.matrix(),RotationX);
    Affine3f ObsePosition(RotationX,Vec3f(0,20,-3));
    mainWindow.setWindowSize(Size(Cam.getWindowSize().width/2,Cam.getWindowSize().height)); // 设置主窗口大小


    // Display camera in window
    mainWindow.showWidget("Cam",camOrient_main,camPosition); // 显示相机
    camWindow.setCamera(Cam); // 设置相机窗口的观测相机内参数
    camWindow.showWidget("Camera",camOrient_cam,camPosition);


    // Display an Image
    Mat image = imread("./chessboard.png",0); // 读入图像(调整路径)
    viz::WImage3D Image_main(image,Size(3,2));viz::WImage3D Image_cam(image,Size(3,2)); // 设置图像
    mainWindow.showWidget("Image_image",Image_main,Affine3f::Identity());
    camWindow.showWidget("Image_cam",Image_cam,Affine3f::Identity());

    camWindow.setViewerPose(camPosition); // 设置观测位置
    mainWindow.setViewerPose(ObsePosition);

    namedWindow("Corner_Image",WINDOW_NORMAL); // 角点可视化窗口名称
    while(!mainWindow.wasStopped()){
        mainWindow.showWidget("Coordinate_main",viz::WCoordinateSystem(),Affine3f::Identity()); // 显示坐标系
        camWindow.showWidget("Coordinate_cam",viz::WCoordinateSystem(),Affine3f::Identity()); // 显示坐标系
        camWindow.setRenderingProperty("Coordinate_cam",viz::OPACITY,0.4); // 设置相机窗口下的坐标系的显示参数(透明度)


        camWindow.setWidgetPose("Camera",camWindow.getViewerPose()); // Refresh Pose in camWindow 相机位置与观测位姿重合，这样屏幕上的图像就是相机实际获取的图像
        // 抵消相机平移距离的影响
        Vec3f tmpTrans = camWindow.getViewerPose().inv().translation(); // 位姿直接取反，平移向量会变成负值，需要单独计算
        tmpTrans += camPosition.translation();

        //  Detect Corner and draw 检测并绘制角点
        Mat tmpCornerImage = camWindow.getScreenshot();
        Mat DrawCorner = tmpCornerImage.clone();
        vector<Point2f> corner;
        bool find = findChessboardCorners(tmpCornerImage,Size(11,8),corner,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
        if(find){
            drawChessboardCorners(DrawCorner,Size(11,8),corner,true);
            imshow("Corner_Image",DrawCorner);
            waitKey(50);
        }else{
            putText(DrawCorner,"Couldn't Find Corner",Point(DrawCorner.rows/2,DrawCorner.cols/2),1,2,Scalar(0,0,255),4,LINE_8,false);
            imshow("Corner_Image",DrawCorner);
            waitKey(50);

        }


        mainWindow.setWidgetPose("Image_image",Affine3f(camWindow.getViewerPose().rotation().inv(),tmpTrans)); // 更新主窗口下标定板的位姿

        mainWindow.spinOnce(1,true); // 循环刷新窗口
        camWindow.spinOnce(1,true);

    }
    cout<<"Hello World"<<endl;
    return 0;
}

