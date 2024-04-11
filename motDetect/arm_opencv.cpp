#include <opencv2/opencv.hpp>
#include <iostream>
#include "arm_sys.h" // 包含机械臂的头文件

using namespace cv;
using namespace std;

int main() {
    VideoCapture capture(0); // 打开默认摄像头
    if (!capture.isOpened()) {
        cerr << "Error opening video capture." << endl;
        return -1;
    }

    Mat frame;
    mg90s servo(17); // 假设伺服电机的GPIO pin是17

    while (capture.read(frame)) {
        if (frame.empty()) {
            cerr << "No captured frame -- Break!" << endl;
            break;
        }

        Mat hsv, redMask1, redMask2, redMask;
        cvtColor(frame, hsv, COLOR_BGR2HSV); // 转换到HSV色彩空间
        inRange(hsv, Scalar(0, 70, 50), Scalar(10, 255, 255), redMask1);
        inRange(hsv, Scalar(170, 70, 50), Scalar(180, 255, 255), redMask2);
        redMask = redMask1 | redMask2; // 合并两个红色范围的掩模

        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(redMask, redMask, MORPH_CLOSE, kernel); // 使用形态学操作改善掩模结果

        vector<vector<Point>> contours;
        findContours(redMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        Rect objectBoundingRectangle = Rect(0, 0, 0, 0);
        if (!contours.empty()) {
            vector<Point> largestContour = contours[0];
            for (size_t i = 1; i < contours.size(); i++) {
                if (contourArea(contours[i]) > contourArea(largestContour)) {
                    largestContour = contours[i];
                }
            }
            objectBoundingRectangle = boundingRect(largestContour);
            rectangle(frame, objectBoundingRectangle, Scalar(0, 255, 0), 2);
        }

        int targetX = objectBoundingRectangle.x + objectBoundingRectangle.width / 2; // 目标中心X坐标
        int centerX = frame.cols / 2;
        int deviation = targetX - centerX; // 计算偏差

        float angle = deviation * 90.0 / centerX; // 假设屏幕宽度对应-90到90度
        servo.setTargetAngleAsync(angle, []() {
            cout << "Target angle reached." << endl;
        });

        imshow("Red Object Detection", frame); // 显示结果

        if (waitKey(10) == 27) {
            break; // 按 'ESC' 键退出
        }
    }

    capture.release(); // 释放资源
    destroyAllWindows();
    return 0;
}
