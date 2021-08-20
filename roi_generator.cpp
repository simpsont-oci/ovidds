#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O

#include <stdio.h>
#include <iostream>

const std::string WindowName("Face Detection");

using namespace std;
using namespace cv;

class CascadeDetectorAdapter : public DetectionBasedTracker::IDetector
{
public:
  CascadeDetectorAdapter(cv::Ptr<cv::CascadeClassifier> detector):
    IDetector(),
    Detector(detector)
  {
    CV_Assert(detector);
  }

  void detect(const cv::Mat &Image, std::vector<cv::Rect> &objects) 
  {
      Detector->detectMultiScale(Image, objects, scaleFactor, minNeighbours, 0, minObjSize, maxObjSize);
  }

  virtual ~CascadeDetectorAdapter() 
  {}

private:
  CascadeDetectorAdapter();
  cv::Ptr<cv::CascadeClassifier> Detector;
};

int main(int , char** )
{
  namedWindow(WindowName);

  VideoCapture capture;
  capture.open(0);
  //capture.set(CAP_PROP_FRAME_WIDTH, 1280);
  //capture.set(CAP_PROP_FRAME_HEIGHT, 720);

  if (!capture.isOpened())
  {
      printf("Error: Cannot open video stream from camera\n");
      return 1;
  }

  std::string cascadeFrontalfilename1 = "/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml";
  cv::Ptr<cv::CascadeClassifier> cascade1 = makePtr<cv::CascadeClassifier>(cascadeFrontalfilename1);
  cv::Ptr<DetectionBasedTracker::IDetector> MainDetector1 = makePtr<CascadeDetectorAdapter>(cascade1);
  if (cascade1->empty() )
  {
    printf("Error: Cannot load %s\n", cascadeFrontalfilename1.c_str());
    return 2;
  }

  cascade1 = makePtr<cv::CascadeClassifier>(cascadeFrontalfilename1);
  cv::Ptr<DetectionBasedTracker::IDetector> TrackingDetector1 = makePtr<CascadeDetectorAdapter>(cascade1);
  if (cascade1->empty() )
  {
    printf("Error: Cannot load %s\n", cascadeFrontalfilename1.c_str());
    return 2;
  }

  DetectionBasedTracker::Parameters params1;
  DetectionBasedTracker Detector1(MainDetector1, TrackingDetector1, params1);

  if (!Detector1.run())
  {
      printf("Error: Detector initialization failed\n");
      return 2;
  }

  std::string cascadeFrontalfilename2 = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml";
  cv::Ptr<cv::CascadeClassifier> cascade2 = makePtr<cv::CascadeClassifier>(cascadeFrontalfilename2);
  cv::Ptr<DetectionBasedTracker::IDetector> MainDetector2 = makePtr<CascadeDetectorAdapter>(cascade2);
  if (cascade2->empty() )
  {
    printf("Error: Cannot load %s\n", cascadeFrontalfilename2.c_str());
    return 2;
  }

  cascade2 = makePtr<cv::CascadeClassifier>(cascadeFrontalfilename2);
  cv::Ptr<DetectionBasedTracker::IDetector> TrackingDetector2 = makePtr<CascadeDetectorAdapter>(cascade2);
  if (cascade2->empty() )
  {
    printf("Error: Cannot load %s\n", cascadeFrontalfilename2.c_str());
    return 2;
  }

  DetectionBasedTracker::Parameters params2;
  DetectionBasedTracker Detector2(MainDetector2, TrackingDetector2, params2);

  if (!Detector2.run())
  {
      printf("Error: Detector initialization failed\n");
      return 2;
  }

  Mat ReferenceFrame;
  Mat GrayFrame;
  vector<Rect> Faces1;
  vector<Rect> Faces2;

  while (1)
  {
    capture >> ReferenceFrame;
    cvtColor(ReferenceFrame, GrayFrame, COLOR_BGR2GRAY);

    Detector1.process(GrayFrame);
    Detector1.getObjects(Faces1);

    Detector2.process(GrayFrame);
    Detector2.getObjects(Faces2);

    for (size_t i = 0; i < Faces1.size(); i++)
    {
      rectangle(ReferenceFrame, Faces1[i], Scalar(0,255,0));
    }

    for (size_t i = 0; i < Faces2.size(); i++)
    {
      rectangle(ReferenceFrame, Faces2[i], Scalar(0,0,255));
    }

    imshow(WindowName, ReferenceFrame);
    char c = static_cast<char>(waitKey(10));
  
    // Press q to exit from window
    if (c == 27 || c == 'q' || c == 'Q') 
      break;
  }

  Detector1.stop();

  Detector2.stop();

  return 0;
}

