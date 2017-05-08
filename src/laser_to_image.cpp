/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, HaoChih, LIN
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <laser_geometry/laser_geometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class LaserToImage
{
    public:
        LaserToImage();

    private:
        ros::NodeHandle nh_;                          
        ros::Subscriber scan_sub;			     
        ros::Publisher img_pub;  
        laser_geometry::LaserProjection projector_;                 
        void scan_callback(const sensor_msgs::LaserScanConstPtr& scan_in);
};

LaserToImage::LaserToImage()
{
    scan_sub = nh_.subscribe("/scan", 10, &LaserToImage::scan_callback, this);
    img_pub = nh_.advertise<sensor_msgs::Image>("/lidargrid",10, true);
}

void LaserToImage::scan_callback(const sensor_msgs::LaserScanConstPtr& scan_in)
{
    sensor_msgs::PointCloud::Ptr cloud(new sensor_msgs::PointCloud);
    projector_.projectLaser(*scan_in, *cloud);

    int width = 100;
    cv::Mat grey = cv::Mat::zeros(width, width, CV_8UC1);

    for (int i=0; i < cloud->points.size(); i++)
    {
        geometry_msgs::Point32 pt = cloud->points[i];
        double x = (double)pt.x;
        double y = (double)pt.y;
        int xPixel = (int)(((x+5.0)/10.0)*width);
        int yPixel = (int)(((y+5.0)/10.0)*width);
        if(xPixel >= 0 && xPixel < width && yPixel >= 0 && yPixel < width) 
            grey.at<uint8_t>(xPixel,yPixel)=255;
    }

    cv_bridge::CvImage out_msg;
    out_msg.header = scan_in->header;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    out_msg.image = grey;
    img_pub.publish(out_msg);
}


//------------Main function-----------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser2Image");
  ROS_INFO("=== Laser To Image Node Start ===");  
  LaserToImage Laser2Image;

  while( ros::ok() )
    ros::spin();

  return EXIT_SUCCESS;
}
