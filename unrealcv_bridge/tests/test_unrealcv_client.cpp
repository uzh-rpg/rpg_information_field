#include "unrealcv_bridge/unrealcv_client.hpp"

#include <opencv2/highgui/highgui.hpp>

using boost::asio::ip::tcp;
using namespace std;

int main()
{
  unrealcv_bridge::UnrealCvClient client("localhost", "9000");

  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Depthmap", cv::WINDOW_AUTOSIZE);

  for (double z = 92.0; z < 100.0; z += 0.5)
  {
    unrealcv_bridge::CameraData test = { 0,
                                          0.000,20.001, 0.000,
                                          -1605.391, -6403.528, z };

    client.setCamera(test);
    cv::Mat img = client.getImage(0);
    cv::imshow("Image", img);

    cv::Mat depthmap = client.getDepth(0);
    cv::normalize(depthmap, depthmap, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imshow("Depthmap", depthmap);

    cv::waitKey(1000);
  }

  return 0;
}
