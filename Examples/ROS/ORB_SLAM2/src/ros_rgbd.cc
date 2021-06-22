/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"



using namespace std;
int lp11=1;
double lp12=0;
int flags=0;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    cerr << endl << "start" << endl;
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    //cerr << endl << "stage:2" << endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    //cerr << endl << "stage:3" << endl;
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
  
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh,"/kinect2/qhd/image_color",1);
message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/kinect2/qhd/image_depth_rect",1);

    //cerr << endl << "stage:4" << endl;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    //cerr << endl << "stage:5" << endl;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    //cerr << endl << "stage:6" << endl;
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    //cerr << endl << "stage:7" << endl;

    //cerr << endl << "stage:7.1 " << endl;
     ros::Time begintime=ros::Time::now();
     ros::Time old=ros::Time::now();
    cerr << endl << "stage:7.1 start time" <<begintime.toSec()<< endl;


    //ros::spin();
    //while(flags==0){ros::spinOnce();}
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
     ros::Rate loop_rate(5);
    // while (ros::ok())
    // {
    //     if(flags==1)break;
    //     /*...TODO...*/ 
 
    //     ros::spinOnce();
    //     loop_rate.sleep();
        
    // }
    ros::spin();

    cerr << endl << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1end and time is:" << lp12<<endl;
    begintime=ros::Time::now();
    ros::Time mid=ros::Time::now();
    cerr << endl <<old<< "stage:7.1 start time" <<old.toSec()<< endl;
    cerr << endl <<old<< "时间差1：" <<begintime-old<< endl;
   cerr << endl <<old<< "帧率：" <<lp12/(begintime.toSec()-old.toSec())<< endl;

    cerr << endl << "stage:8.1 time:" <<begintime<< endl;



    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   
    SLAM.SaveMap();
    begintime=ros::Time::now();
    cerr << endl << "finish time:" <<begintime<< endl;
    cerr << endl <<old<< "时间差2：" <<begintime-old<< endl;
    cerr << endl <<old<< "时间差3：" <<begintime-mid<< endl;
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrDD;
    
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        cv::imwrite("/home/finch/data/rgb/"+to_string(lp11)+".jpg",cv_ptrRGB->image);
        
        cv_ptrDD = cv_bridge::toCvShare(msgD);
        vector< int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back( 0);
        cv::imwrite("/home/finch/data/depth/"+to_string(lp11)+".png",cv_ptrDD->image,compression_params);
        
    
    //sprintf(picadr,"/home/finch/data/argb/%d.jpg",lp11);
    // try
    // {
    //     /* code */
    //     throw cv::imread("/home/finch/data/black.jpg", 0);

    // }
    // catch(cv::Mat)
    // {
    //     std::cerr << e.what() << '\n';
    // }




    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cerr << " error1" << endl;
    
    
    // ros::Time begintime=ros::Time::now();
    // ros::Time old;
    //cerr << endl << "stage:7.1" <<begintime.toSec()<<" time "<<cv_ptrRGB->header.stamp<< endl;

    //mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_dptrD->header.stamp.toSec(),lp11);
    // if(lp11++>=0){lp11=0;//cerr << " time： "<<++lp12<< endl;
    // ++lp12;

    cerr << " wait!" << endl;
    cv::Mat black;
    cv::Mat adepth;
    
    black=cv::imread("/home/finch/data/argb/"+to_string(lp11)+".jpg",1);
    while(black.data== nullptr){
        black=cv::imread("/home/finch/data/argb/"+to_string(lp11)+".jpg", 1);
    }
    cerr << " receiveddepth:"<<lp11 << endl;
    adepth=cv::imread("/home/finch/data/adepth/"+to_string(lp11)+".png",-1);
    cerr << " receivedargb"<<lp11 << endl;
    black=cv::imread("/home/finch/data/argb/"+to_string(lp11)+".jpg",1);

    // if(lp11==1)
    // {cerr << " rider kick" << endl;
    // cerr << " 3..." << endl;
    // cerr << " 2...." << endl;
    // cerr << " 1....." << endl;
    
    // black=cv::imread("/home/finch/data/argb/"+to_string(lp11)+".jpg", 1);}
    // else
    // black=cv::imread("/home/finch/data/argb/"+to_string(lp11-1)+".jpg", 1);



    
    
    
    //black=cv::imread("/home/finch/data/argb/"+to_string(lp11-1)+".jpg", 1);
    // if(!mpSLAM->TrackRGBD(black,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),lp12).data){
    //    flags=1;
    //     lp11++;
    // }
    mpSLAM->TrackRGBD(black,adepth,cv_ptrRGB->header.stamp.toSec(),lp11).data;
       //flags=1;
        
    lp11++;
    // }
    // if(lp12>=99999)lp12=0;
    //if(lp12>=4){
     //lp11=0;
    //}
    //cerr << endl << "stage:7.2" <<begintime<<" time "<<cv_ptrRGB->header.stamp<< endl;
}


