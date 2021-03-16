#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <camera_info_manager/GetCamerainfo.h>
#include <iostream>

class Astra_param
{
    public:
        float fx;
        float fy;
        float cx;
        float cy;

        int width;
        int height;
    Astra_param()
    {
        fx = 577.9669799804688f;
        fy = 577.9669799804688f;
        cx = 328.6839904785156f;
        cy = 239.28900146484375f;
        width = 640;
        height = 480;
    }
    ~Astra_param()
    {}
};

class camera_manager
{
public:
    Astra_param astra_;

    std::string frame_id_;
    std::string distortion_model_;
    ros::NodeHandle nh_;
    ros::ServiceServer service;

    bool Publish_Data(camera_info_manager::GetCamerainfoRequest& req, camera_info_manager::GetCamerainfoResponse& res)
    {
        ros::Time sensor_time = ros::Time::now();

        sensor_msgs::CameraInfo info;
        info.D.resize(5, 0.0);
        info.D[0] = 0.0f;
        info.D[1] = 0.0f;
        info.D[2] = 0.0f;
        info.D[3] = 0.0f;
        info.D[4] = 0.0f;        

        info.K.assign(0.0);
        info.K[0] = astra_.fx;
        info.K[2] = astra_.cx;
        info.K[4] = astra_.fy;
        info.K[5] = astra_.cy;
        info.K[8] = 1.0f;

        info.P.assign(0.0);
        info.P[0] = info.K[0];
        info.P[2] = info.K[2];
        info.P[3] = 0.0f;
        info.P[5] = info.K[4];
        info.P[6] = info.K[5];
        info.P[7] = 0.0f;
        info.P[10] = 1.0f;

        info.height = astra_.height;
        info.width = astra_.width;
        info.distortion_model = distortion_model_;
        info.header.stamp = sensor_time;
        info.header.frame_id = frame_id_; 

        res.info = info;
        return true;
    }
    
    camera_manager()
    {
        frame_id_ = "camera";
        distortion_model_ = "plumb_bob";
        service = nh_.advertiseService("camera_info", &camera_manager::Publish_Data, this);

    }
    ~camera_manager()
    {}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_manager");

    camera_manager cam_info;
    ros::spin();
    return 0;
}