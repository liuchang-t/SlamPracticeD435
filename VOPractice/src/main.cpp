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
        // û������������Ǿ������õĲ���
        myslam::Config::setParameterFile("C:/Users/LiuChang/source/repos/SLAM/SlamPracticeD435/VOPractice/config/default.yaml");
    }
    else if (argc == 2)
    {
        // �������Ϊ�����ļ�����·����������
        myslam::Config::setParameterFile(argv[1]);
    }
    else
    {
        cout << "usage: run_vo parameter_file" << endl;
        return 1;
    }

    // visualization
    // ʹ��vizģ�鴴�����ӻ����ڣ�����ʼ����������ϵ���������ϵ����ʼ�ӽǵȵ�
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);

    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("World", world_coor);
    vis.showWidget("Camera", camera_coor);

    // �����ʼ��
    rs2::context ctx;
    auto list_dev = ctx.query_devices();
    auto list_sen = ctx.query_all_sensors();
    if ((list_dev.size() == 0)||(list_sen.size() == 0))
    {
        cout << "δ��⵽����򴫸�����������ֹ" << endl;
        cout << "Press any key to exit..." << endl;
        getchar();
        return 0;
    }
    int width = 640;    // ����ͼ��ֱ��ʺ�֡��
    int height = 480;
    int fps = 15;
    rs2::config cfg;    
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);    //  �������ͼ��RGBͼ�����ĸ�ʽ
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);
    // ��ȡ��ɫ�����ͼ�������ö���
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    // ��ȡ����ڲ�
    auto i = depth_stream.get_intrinsics();
    // ��ȡ��������depth_scale��ע�������depth_scale����Ϊ������depth_scale
    rs2::device selected_device = profile.get_device();    // select the started device
    auto depth_sensor = selected_device.first<rs2::depth_sensor>(); 
    float depth_scale = depth_sensor.get_depth_scale();
    // �����Զ��������Ĳ���������������depth_scale����Ϊ��������ģ���������Ҫ�������
    myslam::Camera::Ptr camera(new myslam::Camera(i.fx, i.fy, i.ppx, i.ppy, 1.0/depth_scale));

    const auto window_name = "Display Image";

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);
    int num = 0;
    //while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    while (waitKey(1) < 0)
    {
        rs2::frameset frameset = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        // �������ͼ��RGBͼ�ļ��ã���Ϊ���ͼ��RGBͼ��Ұ�������ֶ�������ͼ��ߴ磬����Ͳ���Ҫ��
        // rs2::align align_to_color(RS2_STREAM_COLOR);
        // frameset = align_to_color.process(frameset); 

        rs2::frame rs_depthFrame = frameset.get_depth_frame();
        rs2::frame rs_colorFrame = frameset.get_color_frame();

        const int w_depth = rs_depthFrame.as<rs2::video_frame>().get_width();
        const int h_depth = rs_depthFrame.as<rs2::video_frame>().get_height();

        const int w_color = rs_colorFrame.as<rs2::video_frame>().get_width();
        const int h_color = rs_colorFrame.as<rs2::video_frame>().get_height();

        Mat depth(Size(w_depth, h_depth), CV_16UC1, (void*)rs_depthFrame.get_data(), Mat::AUTO_STEP);
        Mat color(Size(w_color, h_color), CV_8UC3, (void*)rs_colorFrame.get_data(), Mat::AUTO_STEP);
        if (color.data == nullptr || depth.data == nullptr)
            break;

        // ����ȡ����ͼ�����ݺ�ʱ�������Ϊһ�� Frame ֡����
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;

        //cv::imshow("color", color);
        //cv::moveWindow("color", 640, 520);

        // ��������ոմ򿪵�ʱ������ļ�֡ͼ�������⣬����ǰ10֡�Ȳ�Ҫ
        if (num < 10)
        {
            num++;
            continue;
        }

        // boost::timer timer_1;
        // �� VisualOdometry �����м���� Frame ֡
        // addFrame()��VisualOdometry����Ҫ�������������������ȡ�������Ӽ��㡢ͼ��ƥ�䡢λ�˹��ơ��ο�֡���ݸ��µ�һϵ������
        vo->addFrame(pFrame);
        // cout<<"VO costs time: "<<timer_1.elapsed()<<endl;

        // ��һ�μ��飬���vo���ڸ�����״̬�����˳�ѭ��
        if (vo->state_ == myslam::VisualOdometry::LOST)
            break;
        // ���vo��û�и������Ǿͻ�ȡ��ǰ�������������ϵ�е�λ�ˣ���addFrame()��ʱ�򣬻���µ�ǰ֡��Tcw����Tcw���������
        // ����������ϵ���������ϵ��λ�˱任����������ת�����ƽ��������ɣ�������ת�����������������ϵ���������ϵ�µ���̬��
        // ��ƽ���������������������ϵ��ԭ�����������ϵ�е����ꡣ
        // ��ˣ���Tcw������󣬾Ϳ��Եõ��������ϵ����������ϵ�µ�λ�ˡ�
        SE3d Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose 
        // ��Ҫʹ��cv::Affine3��������vizģ������ϵλ�õĸ��£�cv::Affine3����ĺ�����ʵ����һ��4*4��λ�˱任���󣬸����䶨�壬
        // ����ʹ����ת����R��ƽ������t��Ϊ�����������������ֻ��������Ҫ�ľ�����������£�
        // ����SE3d
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

        // ��������զ��ɫͼ���ϱ�ǳ���
        Mat img_show = color.clone();
        for (auto& pt : vo->map_->map_points_)
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel(p->pos_, pFrame->T_c_w_);
            cv::circle(img_show, cv::Point2f(pixel(0, 0), pixel(1, 0)), 5, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow(window_name, img_show);   // ��ʾ�����������ǵ�ͼ��
        cv::namedWindow(window_name, WINDOW_AUTOSIZE);
        cv::moveWindow(window_name, 0, 520);
        cv::waitKey(1); 
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(1, false);
    }

    return 0;
}