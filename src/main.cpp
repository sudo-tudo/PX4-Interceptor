#include <iostream>
#include <vector>
#include <chrono>

#include "common/common_utils/StrictMode.hpp"
#include "PIDController.hpp"

STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK msgpack
#endif
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/ImageCaptureBase.hpp"
STRICT_MODE_ON

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>

int main() {
    using namespace msr::airlib;

    MultirotorRpcLibClient client("127.0.0.1"); 
    client.confirmConnection();

    client.enableApiControl(true, "Interceptor");
    client.armDisarm(true, "Interceptor");
    client.takeoffAsync(5, "Interceptor")->waitOnLastTask();

    std::cout << "Connected! Press ESC in the 'Drone Eyes' window to exit." << std::endl;
    cvNamedWindow("Drone Eyes", CV_WINDOW_AUTOSIZE);
    CvFont overlay_font;
    cvInitFont(&overlay_font, CV_FONT_HERSHEY_SIMPLEX, 0.6, 0.6, 0.0, 2, CV_AA);

    PIDController pid(0.1f, 0.005f, 1.0f);

    while (true) {
        const std::vector<ImageCaptureBase::ImageRequest> request = {
            ImageCaptureBase::ImageRequest("front_center", ImageCaptureBase::ImageType::Scene, false, false)
        };
        std::cout<< "Created request" << std::endl;


        const std::vector<ImageCaptureBase::ImageResponse> response = client.simGetImages(request);
        std::cout<< "Got response" << std::endl;

        if (response.size() > 0) {
            cv::Mat frame(response[0].height, response[0].width, CV_8UC3, (void*)response[0].image_data_uint8.data());
            
            if (!frame.empty()) {
                cv::Mat hsv, mask, mask1, mask2;

                cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

                cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
                cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);

                mask = mask1 | mask2;
                
                cv::Moments m = cv::moments(mask, true);
                IplImage frame_ipl = cvIplImage(frame);

                if (m.m00 >= 100.0) {
                    int cx = static_cast<int>(m.m10 / m.m00);
                    int cy = static_cast<int>(m.m01 / m.m00);
                
                    cv::drawMarker(frame, cv::Point(cx, cy), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 30, 2);
                    int error_x = cx - 640;
                    int error_y = cy - 360;

                    pid.InitiateControl(error_x, error_y, std::move(m), client);

                    std::string status = "LOCK: [" + std::to_string(error_x) + ", " + std::to_string(error_y) + "]";
                    cvPutText(&frame_ipl, status.c_str(), cvPoint(cx + 20, cy - 20), &overlay_font, cvScalar(0, 255, 0, 0));
                } else {
                    cvPutText(&frame_ipl, "SEARCHING...", cvPoint(50, 50), &overlay_font, cvScalar(0, 0, 255, 0));
                }
                cvShowImage("Drone Eyes", &frame_ipl);
            }
        }

        if (cvWaitKey(1) == 27) break; 
    }

    cvDestroyWindow("Drone Eyes");
    return 0;
}
