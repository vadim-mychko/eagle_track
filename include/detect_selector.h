// This file is part of OpenCV project (which was modified, see cv::selectROI).
// It is subject to the license terms in the LICENSE file found at https://github.com/opencv/opencv/blob/4.4.0/LICENSE.
#include <opencv2/opencv.hpp>

using namespace cv;

namespace
{

class DetectSelector {
public:
  struct handlerT {
    Mat image;
    std::vector<Point2f> points;
  } selectorParams;

  std::vector<Point2f> select(const String& windowName, Mat img, bool printNotice);

private:
  // save the keypressed character
  int key;

  static void emptyMouseHandler(int, int, int, int, void *);
  static void mouseHandler(int event, int x, int y, int flags, void *param);
  void opencv_mouse_callback(int event, int x, int y, int);
};

std::vector<Point2f> DetectSelector::select(const String& windowName, Mat img, bool printNotice) {
  if (printNotice) {
    // show notice to user
    printf("Select a ROI and then press SPACE or ENTER button!\n");
    printf("Cancel the selection process by pressing c button!\n");
  }

  key = 0;
  // set the drawing mode
  // show the image and give feedback to user
  imshow(windowName, img);
  // copy the data, rectangle should be drawn in the fresh image
  selectorParams.image = img.clone();
  // select the object
  setMouseCallback(windowName, mouseHandler, (void *)this);
  // end selection process on SPACE (32) ESC (27) or ENTER (13)
  while (!(key == 32 || key == 27 || key == 13)) {
    // draw the selected object
    for (auto& point : selectorParams.points) {
      circle(selectorParams.image, point, 5, Scalar(255, 0, 0), -1);
    }

    imshow(windowName, selectorParams.image);
    // reset the image
    selectorParams.image = img.clone();
    // get keyboard event
    key = waitKey(30);
    if (key == 'c' || key == 'C') // cancel selection
    {
      selectorParams.points.clear();
      break;
    }
  }

  // cleanup callback
  setMouseCallback(windowName, emptyMouseHandler, NULL);
  return selectorParams.points;
}

void DetectSelector::emptyMouseHandler(int, int, int, int, void *) {}

void DetectSelector::mouseHandler(int event, int x, int y, int flags, void *param) {
  auto self = static_cast<DetectSelector*>(param);
  self->opencv_mouse_callback(event, x, y, flags);
}

void DetectSelector::opencv_mouse_callback(int event, int x, int y, int) {
  if (event == EVENT_LBUTTONDOWN) {
    selectorParams.points.push_back({static_cast<float>(x), static_cast<float>(y)});
  }
}

}; // namespace

std::vector<Point2f> selectPoints(const String& windowName, InputArray img, bool printNotice = true) {
  DetectSelector selector;
  return selector.select(windowName, img.getMat(), printNotice);
}
