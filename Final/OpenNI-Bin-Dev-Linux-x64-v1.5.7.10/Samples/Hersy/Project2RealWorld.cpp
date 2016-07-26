// Project2RealWorld.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include <iostream>
#include <iomanip>
#include <fstream> /* ifstream class */
#include <string>  /* string class   */
#include <cstdlib> /* atoi   */
#include <sstream> /* istringstream class */
using namespace std;

fstream file;
const int image_number = 9;
char *fileName[image_number] = {"RealWordData_Img_8.txt","RealWordData_Img_11.txt","RealWordData_Img_12.txt",
								"RealWordData_Img_13.txt", "RealWordData_Img_14.txt", "RealWordData_Img_15.txt", "RealWordData_Img_16.txt", "RealWordData_Img_17.txt", "RealWordData_Img_18.txt"};
char *depthImgName[image_number] = {"Image_8.yml","Image_11.yml",
									"Image_12.yml","Image_13.yml", "Image_14.yml", "Image_15.yml", "Image_16.yml", "Image_17.yml", "Image_18.yml"};

int RealNumber[image_number]  = { 8,11,12,13,14,15,16,17,18};
//----OpenNI Include-------
#include <XnCppWrapper.h>

//----OpenCV Include-------
#include <highgui.h>
#include <cvaux.h>
#include <cxcore.h>
#include <cv.h>

xn::Context mContext;
xn::DepthGenerator mDepthGenerator;
xn::ImageGenerator mImageGenerator;
XnMapOutputMode mapMode;
XnPoint3D* pRealWorldCamaraPointSet;

void InitialKinect();
void WriteToFile(XnPoint3D* ,int );

int _tmain(int argc, _TCHAR* argv[])
{
	InitialKinect();
	
	std::stringstream out_yml_filename;
	cv::Mat Read_Depth;
	cv::Mat c16BitDepth;
	

	for (int k = 0; k < image_number; k++)
	{
		out_yml_filename.str( *(depthImgName+k) ); 
		cv::FileStorage fsd( out_yml_filename.str(), cv::FileStorage::READ);		
		fsd["frameCount"] >> c16BitDepth;		
		c16BitDepth.convertTo( Read_Depth, CV_32FC1);
		
		int idxShift, idx;
		XnPoint3D* pDepthPointSet = new XnPoint3D[ 640 * 480 ];
		pRealWorldCamaraPointSet = new XnPoint3D[ 640 * 480 ];
		for ( int j = 0; j < 480; ++j )
		{
			idxShift = j * 640;
			for ( int i = 0; i < 640; ++i)
			{
					idx = idxShift + i;
					pDepthPointSet[idx].X = (int)i;
					pDepthPointSet[idx].Y = (int)j;
					pDepthPointSet[idx].Z = (int)Read_Depth.at<float>(idx);
			}
		}
		mDepthGenerator.ConvertProjectiveToRealWorld( 640 * 480, pDepthPointSet, pRealWorldCamaraPointSet );

		WriteToFile(pRealWorldCamaraPointSet, k);
		cout << "Image " <<  RealNumber[k] << " Done!!\n";

		fsd.release();
		delete [] pDepthPointSet;
		delete [] pRealWorldCamaraPointSet;
	}


	return 0;
}



void InitialKinect()
{
	    //1. Initial Context
		mContext.Init();

		// 2. Set Map_mode (建構函數已定義)
		mapMode.nXRes = 640;
		mapMode.nYRes = 480;
		mapMode.nFPS = 30;

		// 3.a Create Depth_Generator
		mDepthGenerator.Create( mContext );
		mDepthGenerator.SetMapOutputMode( mapMode );

		// 3.b Create Image_Generator
		mImageGenerator.Create( mContext );
		mImageGenerator.SetMapOutputMode( mapMode );

		 // 4. Correct view port
		mDepthGenerator.GetAlternativeViewPointCap().SetViewPoint( mImageGenerator );
		std::cout << "正確：KinectSensor Intial Correct!" << std::endl;
}


void WriteToFile(XnPoint3D* pRealWorldCamaraPointSet, int number)
{
	int idxShift, idx;
	file.open( *(fileName + number), ios::out );

	cout << setprecision(3) << fixed;

	for (int i = 0; i < 480; ++i)
	{
		
		idxShift = i * 640;
		for (int j = 0; j < 640; ++j )
		{
			idx = idxShift + j;
			file << setw(5) << idx << '\t' << setw(10) << pRealWorldCamaraPointSet[idx].X << '\t' << setw(10) << pRealWorldCamaraPointSet[idx].Y << '\t' << setw(10) << pRealWorldCamaraPointSet[idx].Z << '\n';
		}
	}

	file.close();
}