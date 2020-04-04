// -------------- test the visual odometry -------------
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
//#include <boost/timer/timer.hpp>
#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include <librealsense2/rs.hpp>


int main(int argc, char** argv)
{
    cout << "argc = " << argc << endl;
    if (argc == 1)
    {
        // 没有输入参数，那就用内置的参数
        myslam::Config::setParameterFile("C:/Users/LiuChang/source/repos/SLAM/SlamPracticeD435/VOPractice/config/default.yaml");
    }
    else if (argc == 2)
    {
        // 输入参数为参数文件完整路径包括名称
        myslam::Config::setParameterFile(argv[1]);
    }
    else
    {
        cout << "usage: run_vo parameter_file" << endl;
        return 1;
    }

    // visualization
    // 使用viz模块创建可视化窗口，并初始化世界坐标系、相机坐标系、初始视角等等
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);

    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("World", world_coor);
    vis.showWidget("Camera", camera_coor);

    // 相机初始化
    rs2::context ctx;
    auto list_dev = ctx.query_devices();
    auto list_sen = ctx.query_all_sensors();
    if ((list_dev.size() == 0)||(list_sen.size() == 0))
    {
        cout << "未检测到相机或传感器，程序终止" << endl;
        cout << "Press any key to exit..." << endl;
        getchar();
        return 0;
    }
    int frame_width = myslam::Config::get<int>("frame_width");    // 设置图像分辨率和帧率
    int frame_height = myslam::Config::get<int>("frame_height");
    int frame_fps = myslam::Config::get<int>("frame_fps");
    rs2::config cfg;   
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, frame_width, frame_height, RS2_FORMAT_Y8, frame_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, frame_fps);    //  设置深度图和RGB图像流的格式
    cfg.enable_stream(RS2_STREAM_COLOR, frame_width, frame_height, RS2_FORMAT_BGR8, frame_fps);
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);
    // 获取彩色和深度图像流配置对象
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    // 获取相机内参
    auto i = depth_stream.get_intrinsics();
    // 获取深度相机的depth_scale，注意这里的depth_scale是作为乘数的depth_scale
    rs2::device selected_device = profile.get_device();    // select the started device
    auto depth_sensor = selected_device.first<rs2::depth_sensor>(); 
    float depth_scale = depth_sensor.get_depth_scale();
    // 设置自定义相机类的参数，由于相机类的depth_scale是作为除数运算的，所以这里要求个倒数
    myslam::Camera::Ptr camera(new myslam::Camera(i.fx, i.fy, i.ppx, i.ppy, 1.0/depth_scale));

    // 如果当前的深度传感器支持调节红外光发射器，就把发射器打开或关闭
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        // 设定值为1，打开发射器；为0，关闭发射器
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, myslam::Config::get<float>("emitter_enabled")); 
    }
    // 如果当前的深度传感器支持调节激光器功率，就可以调节
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        // Query min and max values:
        auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER); // 获取激光器功率的允许调节范围
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.min + (range.max - range.min) * myslam::Config::get<float>("laser_power_ratio"));
    }
    // 如果当前的深度传感器支持调节曝光时间，而且自动调节曝光参数功能被关闭，方可手动设定曝光时间
    if ((depth_sensor.supports(RS2_OPTION_EXPOSURE))&&(myslam::Config::get<float>("auto_exposure_mode")==0))
    {
        // 按照比例设置曝光时间，设定任何值都会停止自动调节曝光时间功能
        auto range = depth_sensor.get_option_range(RS2_OPTION_EXPOSURE);
        depth_sensor.set_option(RS2_OPTION_EXPOSURE, range.min + (range.max - range.min)*myslam::Config::get<float>("exposure_ratio")); 
    }

    const auto window_name = "Display Image";

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);
    int num = 0;
    //bool emitter_status = true;
    //int frame_number = 0;
    //while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    while (waitKey(1) < 0)
    {
        //if (emitter_status)
        //{
        //    if (frame_number == 0)
        //        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
        //    frame_number++;
        //    if (frame_number == frame_fps)
        //    {
        //        emitter_status = false;
        //        frame_number = 0;
        //    }  
        //}
        //else
        //{
        //    if (frame_number == 0)
        //        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
        //    frame_number++;
        //    if (frame_number == frame_fps)
        //    {
        //        emitter_status = true;
        //        frame_number = 0;
        //    }
        //}
        rs2::frameset frameset = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        // 设置深度图到RGB图的剪裁，因为深度图比RGB图视野大，由于手动设置了图像尺寸，这里就不需要了
        // rs2::align align_to_color(RS2_STREAM_COLOR);
        // frameset = align_to_color.process(frameset); 

        rs2::frame rs_depthFrame = frameset.get_depth_frame();
        rs2::frame rs_colorFrame = frameset.get_color_frame();
        //rs2::frame ir_frame_left = frameset.get_infrared_frame(1);
        //Mat infrared(Size(frame_width, frame_height), CV_8UC1, (void*)ir_frame_left.get_data(), Mat::AUTO_STEP);
        //cv::imshow("infrared", infrared);
        //cv::moveWindow("infrared", 640, 520);
        Mat depth(Size(frame_width, frame_height), CV_16UC1, (void*)rs_depthFrame.get_data(), Mat::AUTO_STEP);
        Mat color(Size(frame_width, frame_height), CV_8UC3, (void*)rs_colorFrame.get_data(), Mat::AUTO_STEP);
        if (color.data == nullptr || depth.data == nullptr)
            break;

        // 将获取到的图像数据和时间戳整合为一个 Frame 帧对象
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;

        //cv::imshow("color", color);
        //cv::moveWindow("color", 640, 520);

        // 由于相机刚刚打开的时候，最初的几帧图像有问题，所以前10帧先不要
        if (num < 5)
        {
            num++;
            continue;
        }

        // boost::timer timer_1;
        // 向 VisualOdometry 对象中加入该 Frame 帧
        // addFrame()是VisualOdometry的主要函数，其包含了特征提取、描述子计算、图像匹配、位姿估计、参考帧数据更新等一系列流程
        vo->addFrame(pFrame);
        // cout<<"VO costs time: "<<timer_1.elapsed()<<endl;

        // 做一次检验，如果vo处于跟丢的状态，就退出循环
        if (vo->state_ == myslam::VisualOdometry::LOST)
            break;

        // show the map and the camera pose 
        // 需要使用cv::Affine3类来进行viz模块坐标系位置的更新，cv::Affine3对象的核心其实就是一个4*4的位姿变换矩阵，根据其定义，
        // 可以使用旋转矩阵R和平移向量t作为参数来声明这个对象，只不过它需要的矩阵和向量如下：
        // 首先获取Twc
        // 如果vo还没有跟丢，那就获取当前相机在世界坐标系中的位姿，在addFrame()的时候，会更新当前帧的Tcw矩阵，Tcw矩阵代表了
        // 从世界坐标系到相机坐标系的位姿变换矩阵，他由旋转矩阵和平移向量组成，其中旋转矩阵代表了世界坐标系在相机坐标系下的姿态，
        // 而平移向量则代表了世界坐标系的原点在相机坐标系中的坐标。
        // 因此，对Tcw求逆矩阵，就可以得到相机坐标系在世界坐标系下的位姿。
        SE3d Twc = pFrame->T_c_w_.inverse();
        cv::Affine3d M(
            cv::Affine3d::Mat3(
                Twc.rotationMatrix()(0, 0), Twc.rotationMatrix()(0, 1), Twc.rotationMatrix()(0, 2),
                Twc.rotationMatrix()(1, 0), Twc.rotationMatrix()(1, 1), Twc.rotationMatrix()(1, 2),
                Twc.rotationMatrix()(2, 0), Twc.rotationMatrix()(2, 1), Twc.rotationMatrix()(2, 2)
                ),
            cv::Affine3d::Vec3(
                Twc.translation()(0, 0), Twc.translation()(1, 0), Twc.translation()(2, 0)
                )
            );
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(1, false);

        // 将特征点咋彩色图像上标记出来
        Mat img_show = pFrame->color_.clone();
        for (auto& pt : vo->map_->map_points_)
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel(p->pos_, pFrame->T_c_w_);
            cv::circle(img_show, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow(window_name, img_show);   // 显示带有特征点标记的图像
        cv::namedWindow(window_name, WINDOW_AUTOSIZE);
        cv::moveWindow(window_name, 0, 520);
        cv::waitKey(1); 
        
    }

    return 0;
}