#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>  // 包含人脸识别的头文件
#include <stdio.h>

using namespace cv;

int main()
{
    // 加载人脸识别模型，确保 haarcascade_frontalface_default.xml 文件在你的程序目录下或者指定路径中
    CascadeClassifier face_cascade;
    if (!face_cascade.load("haarcascade_frontalface_default.xml")) {
        printf("Error loading face cascade\n");
        return -1;
    }

    VideoCapture capture(0); // 打开默认摄像头
    if (!capture.isOpened()) {
        printf("Error opening video capture\n");
        return -1;
    }

    Mat frame;
    while (capture.read(frame)) {
        if (frame.empty()) {
            printf("No captured frame\n");
            break;
        }

        Mat frame_gray;
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);  // 转换为灰度图，提高处理速度和识别率
        equalizeHist(frame_gray, frame_gray);  // 均衡化处理提高图像质量

        // 检测人脸
        std::vector<Rect> faces;
        face_cascade.detectMultiScale(frame_gray, faces);

        for (size_t i = 0; i < faces.size(); i++) {
            Point center(faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2);
            ellipse(frame, center, Size(faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar(255, 0, 255), 4);
        }

        // 显示结果
        imshow("Face Detection", frame);

        if (waitKey(10) == 27) { // 按 'ESC' 键退出
            break;
        }
    }
    capture.release(); // 释放摄像头资源
    destroyAllWindows(); // 关闭所有 OpenCV 窗口
    return 0;
}
