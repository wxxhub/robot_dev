#include <cstdio>
#include <opencv2/highgui.hpp>

using namespace cv;


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  VideoCapture cap(0);

  if(!cap.isOpened())
	{
		return -1;
	}

  Mat image;

  while (true)
  {
    cap>>image;
    imshow("当前视频",image);
    waitKey(1);
  }


  printf("hello world ball_detector package\n");
  return 0;
}
