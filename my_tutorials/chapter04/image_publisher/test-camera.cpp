#include <iostream>
#include <opencv2/opencv.hpp>
 
using namespace std;
using namespace cv;
 
int main(int argc, char *argv[])
{
    VideoCapture cap(0);
    Mat frame;
     
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(CV_CAP_PROP_FPS, 5);
     
    if(!cap.isOpened()){
        cout << "webcam is not detected" << endl;
        return 1;
    }
    cout << "webcam is detected" << endl;
     
    namedWindow("webcam", 1);
    int key = -1;
    while(key != 'q'){
        cap >> frame;
        imshow("webcam", frame);
        key = waitKey(30);
    }
     
    return 0;
}
