#include "PipelineFrame.h"
#include "PipelineVideo.h"
#include <iostream>
using namespace cv;
using namespace std;


/*int process_video( int argc, const char** argv )
{
    VideoCapture cap;
    cap.open("/Users/xingzh/Library/Mobile Documents/com~apple~CloudDocs/Projects/pipeline-anomaly-detection/res/test_video_1.mp4");
    // Check if camera opened successfully
    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    PipelineVideo pipelineVideo;

    while(1){

        Mat frame;

        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

        Mat resizeFrame;
        double resizeRate = 1;
        cv::resize(frame, resizeFrame, Size(), resizeRate, resizeRate);

        cv::transpose(resizeFrame, resizeFrame);

        Mat greyImg;
        cvtColor(resizeFrame, greyImg, CV_RGB2GRAY);

        pipelineVideo.appendFrame(greyImg);
        pipelineVideo.showBack();

        // Press  ESC on keyboard to exit
        char c=(char)waitKey(25);
        if(c==27)
            break;
    }

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    destroyAllWindows();

    return 0;
}*/

int process_imgs( int argc, const char** argv )
{
    PipelineVideo pipelineVideo;

    for (int i=550; i < 3000; ++i){
        Mat resizeFrame = imread("D:\\git\\12345\\" + to_string(i) + ".jpg");
        if (resizeFrame.empty()) continue;

        Mat greyImg;
        cvtColor(resizeFrame, greyImg, CV_RGB2GRAY);

        pipelineVideo.appendFrame(greyImg);
        pipelineVideo.showBack();

        // Press  ESC on keyboard to exit
        char c=(char)waitKey(25);
        if(c==27)
            break;
    }

    // Closes all the frames
    destroyAllWindows();
    return 0;
}

int main( int argc, const char** argv )
{
//    return process_video(argc, argv);

    return process_imgs(argc, argv);
}