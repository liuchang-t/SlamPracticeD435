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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
//#include <unistd.h>
#include "windows.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "../ORB-SLAM/include/System.h"
#include <librealsense2/rs.hpp>
#include "include/config.h"
#include "include/common_include.h"
using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
    cout << "argc = " << argc << endl;
    myslam::Config c_config;
    if (argc == 1)
    {
        // 没有输入参数，那就用内置的参数
        c_config.setParameterFile("config.yaml");
    }
    else if (argc == 2)
    {
        // 输入参数为参数文件完整路径包括名称
        c_config.setParameterFile(argv[1]);
    }
    else
    {
        cout << "usage: run_vo parameter_file" << endl;
        return 1;
    }

    // 相机初始化
    rs2::context ctx;
    auto list_dev = ctx.query_devices();
    auto list_sen = ctx.query_all_sensors();
    if ((list_dev.size() == 0) || (list_sen.size() == 0))
    {
        cout << "未检测到相机或传感器，程序终止" << endl;
        cout << "Press any key to exit..." << endl;
        getchar();
        return 0;
    }
    int frame_width = c_config.get<int>("Camera.width");    // 设置图像分辨率和帧率
    int frame_height = c_config.get<int>("Camera.height");
    int frame_fps = c_config.get<int>("Camera.fps");
    rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, frame_width, frame_height, RS2_FORMAT_Y8, frame_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, frame_fps);    //  设置深度图和RGB图像流的格式
    cfg.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_BGR8, frame_fps);
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);
    // 获取彩色和深度图像流配置对象
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    
    // 获取相机内参
    auto cam_intrinsics = depth_stream.get_intrinsics();
    // 获取深度相机的depth_scale，注意这里的depth_scale是作为乘数的depth_scale
    rs2::device selected_device = profile.get_device();    // select the started device
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    float depth_scale = 1.0/depth_sensor.get_depth_scale(); //Realsense默认的depth_scale是和ORB-SLAM中用到的是倒数关系

    //// 根据实际相机内参修改参数文件
    //c_config.set("Camera.fx", cam_intrinsics.fx);
    //c_config.set("Camera.fy", cam_intrinsics.fy);
    //c_config.set("Camera.cx", cam_intrinsics.ppx);
    //c_config.set("Camera.cy", cam_intrinsics.ppy);
    //c_config.set("Camera.k1", cam_intrinsics.coeffs[0]);
    //c_config.set("Camera.k2", cam_intrinsics.coeffs[1]);
    //c_config.set("Camera.p1", cam_intrinsics.coeffs[2]);
    //c_config.set("Camera.p2", cam_intrinsics.coeffs[3]);
    //c_config.set("Camera.p3", cam_intrinsics.coeffs[4]);
    //c_config.set("DepthMapFactor", depth_scale);
    //c_config.saveas(ConfigFileName);

    const std::string ORBVoc = c_config.get<string>("ORBvoc");
    const std::string ConfigFileName = c_config.get<string>("ConfigFileName");
    

    // 如果当前的深度传感器支持调节红外光发射器，就把发射器打开或关闭
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        // 设定值为1，打开发射器；为0，关闭发射器
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, c_config.get<float>("EmitterEnabled"));
    }
    // 如果当前的深度传感器支持调节激光器功率，就可以调节
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER); // 获取激光器功率的允许调节范围
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.min + (range.max - range.min) * c_config.get<float>("LaserPowerRatio"));
    }
    // 如果当前的深度传感器支持调节曝光时间，而且自动调节曝光参数功能被关闭，方可手动设定曝光时间
    if ((depth_sensor.supports(RS2_OPTION_EXPOSURE)) && (c_config.get<float>("AutoExposureMode") == 0))
    {
        // 按照比例设置曝光时间，设定任何值都会停止自动调节曝光时间功能
        auto range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
        depth_sensor.set_option(RS2_OPTION_EXPOSURE, range.min + (range.max - range.min) * c_config.get<float>("ExposureRatio"));
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(ORBVoc, ConfigFileName, ORB_SLAM2::System::RGBD, true);

    cout << endl << "-------" << endl;
    cout << "Start ..." << endl;

    // Main loop
    while (waitKey(1) < 0)
    {
        rs2::frameset frameset = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame rs_depthFrame = frameset.get_depth_frame();
        rs2::frame rs_colorFrame = frameset.get_color_frame();
        Mat im_depth(Size(frame_width, frame_height), CV_16UC1, (void*)rs_depthFrame.get_data(), Mat::AUTO_STEP);
        Mat im_color(Size(frame_width, frame_height), CV_8UC3, (void*)rs_colorFrame.get_data(), Mat::AUTO_STEP);
        if (im_color.data == nullptr || im_depth.data == nullptr)
            break;

        double t_start = GetTickCount();

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im_color, im_depth, 0);  // 时间戳应该是用来按照实际节奏加载图片的，实时拍摄的图像应该用不到，先都设为0看看
        //SLAM.TrackRGBD(imRGB, imD, tframe);

        double t_end = GetTickCount();

        double ttrack = t_end - t_start;

    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}


