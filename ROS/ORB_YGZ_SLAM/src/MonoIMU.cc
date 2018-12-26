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
#include<opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include<ros/ros.h>
#include<System.h>
#include "../src/IMU/imudata.h"
#include <mutex>
#include <sensor_msgs/Imu.h>

using namespace std;


#include <stdlib.h>
#include <signal.h>

vector<ygz::IMUData> vImus;
mutex mtx;
void ImuSave(const sensor_msgs::Imu::ConstPtr& imu);

class ImageGrabber
{
public:
    ImageGrabber(ygz::System* pSLAM):mpSLAM(pSLAM){}
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    ygz::System* mpSLAM; 
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoIMU");
    ros::start();
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB-YGZ-SLAM MonoIMU path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ygz::System SLAM(argv[1],argv[2],ygz::System::MONOCULAR,true);
    ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;
    ros::Subscriber img_sub = nh.subscribe("/cam0/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber imu_sub = nh.subscribe("/imu0",100,ImuSave);
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    ros::shutdown();

    return 0;
}

void ImuSave(const sensor_msgs::Imu::ConstPtr& imu)
{
    if(mtx.try_lock())
    {
        double timestamp=imu->header.stamp.toSec();
        ygz::IMUData imudata(imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z,imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z,timestamp);
        vImus.push_back(imudata);
        mtx.unlock();
    }
    
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr & msg)
{
    if(mtx.try_lock())
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        vector<ygz::IMUData> temp;
        int imuIndex = 0;
        double TimeStampPre=0;
        double timestamp=cv_ptr->header.stamp.toSec();
        while(1)
        {
            const ygz::IMUData &imudata = vImus[imuIndex];
            if (imudata._t > timestamp||imudata._t<TimeStampPre)
                break;
            temp.push_back(imudata);
            imuIndex++;
            TimeStampPre=imudata._t;
        }
        vImus.clear();
        mpSLAM->TrackMonoVI(cv_ptr->image,temp,cv_ptr->header.stamp.toSec());
        temp.clear();
        mtx.unlock();
    }
    
}

