#include <iostream>
#include <vector>

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK msgpack
#endif
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/ImageCaptureBase.hpp"
STRICT_MODE_ON

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>

int main() {
    using namespace msr::airlib;

    MultirotorRpcLibClient client("172.17.224.1"); 
    client.confirmConnection();

    std::cout << "Connected! Press ESC in the 'Drone Eyes' window to exit." << std::endl;
    cvNamedWindow("Drone Eyes", CV_WINDOW_AUTOSIZE);

    while (true) {
        const std::vector<ImageCaptureBase::ImageRequest> request = {
            ImageCaptureBase::ImageRequest("front_center", ImageCaptureBase::ImageType::Scene, false, false)
        };

        const std::vector<ImageCaptureBase::ImageResponse> response = client.simGetImages(request);

        if (response.size() > 0) {
            cv::Mat frame = cv::Mat(response[0].height, response[0].width, CV_8UC3, (void*)response[0].image_data_uint8.data());
            
            if (!frame.empty()) {
                IplImage frame_view = cvIplImage(frame);
                cvShowImage("Drone Eyes", &frame_view);
            }
        }

        if (cvWaitKey(1) == 27) break; 
    }

    cvDestroyWindow("Drone Eyes");
    return 0;
}
