#include <stdio.h>
#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define waitTime 1
#define printCorners 1

int main(int argc, char** argv){
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    pipe.start(cfg);
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    int x = 0;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    while (true) {
        frames = pipe.wait_for_frames();
        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        // Creating OpenCV Matrix from a color image
        cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat imageCopy;
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        
        // if at least one marker detected
        if (ids.size() > 0){
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            for(int i=0;i<corners.size();i++){
                float center[2] = {0.f,0.f};
                float planarPoint3d[3];
                rs2_intrinsics intrinsics;
                float pixel_distance_in_meters;

                for(int j=0;j<corners[i].size();j++){
                    auto point = corners[i][j];
                    center[0] += point.x;
                    center[1] += point.y;
                    intrinsics = rs2::video_stream_profile(frames.get_profile()).get_intrinsics();
                    pixel_distance_in_meters = frames.get_depth_frame().get_distance(point.x,point.y);

                    const float pixel[2] = {point.x,point.y};

                    rs2_deproject_pixel_to_point(planarPoint3d, &intrinsics, pixel, pixel_distance_in_meters);
                    if(printCorners){
                    std::cout << "Point " << j << " " << "X: " << point.x << " " << "Y: " << point.y << std::endl;
                    std::cout << "Point " << "X: " << planarPoint3d[0] << "(in meters) " << "Y: " << planarPoint3d[1] << "(in meters) " << "Z: " << planarPoint3d[2] << "(in meters)" << std::endl;
                    }
                }
                for(int i=0;i<2;i++){
                    center[i] = center[i] / 4.0f;
                }
                std::cout << "Center point " << "X: " << center[0] << " " << "Y: " << center[1] << std::endl;
                pixel_distance_in_meters = frames.get_depth_frame().get_distance(center[0],center[1]);
                rs2_deproject_pixel_to_point(planarPoint3d, &intrinsics, center, pixel_distance_in_meters);
                std::cout << "Center point " << "X: " << planarPoint3d[0] << "(in meters) " << "Y: " << planarPoint3d[1] << "(in meters) " << "Z: " << planarPoint3d[2] << "(in meters)" << std::endl;
            }
        }
        cv::imshow("out", imageCopy);
        char key = (char) cv::waitKey(waitTime);        
        if(key == 27)
        break;
    }
}

