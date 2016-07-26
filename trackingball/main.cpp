#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

/*****************************************************************************

1. This function check the type of Mat
2. input  : Mat.type()
3. output : char of ty.c_str()
4. usage  : 
            string ty =  type2str( M.type() );
            printf("Matrix H: %s %dx%d \n", ty.c_str(), M.cols, M.rows );

******************************************************************************/
string type2str(int type) 
{
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

 int main( int argc, char** argv )
 {
    VideoCapture cap; //capture the video from web cam
    cap.open(CV_CAP_OPENNI);

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the camera" << endl;
         return -1;
    }

    int iLowH = 44;
    int iHighH = 78;

    int iLowS = 179; 
    int iHighS = 255;

    int iLowV = 119;
    int iHighV = 177;

    int iLastX = -1; 
    int iLastY = -1;

    //Create trackbars in "Control" window
    /*namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);*/

    while (true)
    {
        Mat imgOriginal;
        Mat imgHSV;
        Mat imgThresholded;
        int count = 0;

        if( !cap.grab() )
        {
            cout << "Can not grab images." << endl;
            return -1;
        }

        cap.retrieve( imgOriginal, CV_CAP_OPENNI_BGR_IMAGE);
        Mat imgLines = Mat::zeros( imgOriginal.rows, imgOriginal.cols, CV_8UC3 );

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //Calculate the moments of the thresholded image
        Moments oMoments = moments(imgThresholded);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
        if (dArea > 10000)
        {
            //calculate the position of the ball
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;        
                
            if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
            {
                //Draw a red line from the previous point to the current point
                //line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
                line(imgLines, Point( 331, 267 ), Point( iLastX,iLastY), Scalar( 255,0, 255 ),  2, 8 );
            }
            imgOriginal = imgOriginal + imgLines;

            iLastX = posX;
            iLastY = posY;
        }

/*
        for (int i = 0; i < 640; ++i)
        {
            for (int j = 0; j < 480; ++j)
            {
                if(imgThresholded.at<int>(i,j) == 255 )
                {
                    count++;
                    if (count == 10)
                    {
                        line( imgOriginal, Point( 331, 267 ), Point( j,i), Scalar( 0,0, 255 ),  2, 8 );
                        //printf("(%d , %d)\n",i,j );
                        count = 0;
                        break;
                    }
                }
            }
        }*/

        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) >= 0) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            string ty =  type2str( imgThresholded.type() );
            printf("imgThresholded : %s %dx%d \n", ty.c_str(), imgThresholded.cols, imgThresholded.rows );
            //cout << imgThresholded << endl;
            break;
        }
    }

   return 0;

}
