#include <stdlib.h>
#include <iostream>
#include <string>
 
#include <XnCppWrapper.h>
  
using namespace std;
 
void CheckOpenNIError( XnStatus eResult, string sStatus )
{
  if( eResult != XN_STATUS_OK )
    cerr << sStatus << " Error : " << xnGetStatusString( eResult ) << endl;
}
 
int main( int argc, char** argv )
{
  XnStatus eResult = XN_STATUS_OK;
 
  // 2. initial context
  xn::Context mContext;
  eResult = mContext.Init();
  CheckOpenNIError( eResult, "initialize context" );
 
  // 3. create depth generator
  xn::DepthGenerator mDepthGenerator;
  eResult = mDepthGenerator.Create( mContext );
  CheckOpenNIError( eResult, "Create depth generator" );

  // 4. create image generator
  xn::ImageGenerator mImageGenerator;
  eResult = mImageGenerator.Create( mContext );
  CheckOpenNIError( eResult, "Create image generator" );

  // 5. set map mode
  XnMapOutputMode mapMode;
  mapMode.nXRes = 640;
  mapMode.nYRes = 480;
  mapMode.nFPS = 30;
  eResult = mDepthGenerator.SetMapOutputMode( mapMode );
  eResult = mImageGenerator.SetMapOutputMode( mapMode );
 
  // 6. correct view port
  mDepthGenerator.GetAlternativeViewPointCap().SetViewPoint( mImageGenerator );
 
  // 7. tart generate data
  eResult = mContext.StartGeneratingAll();
 
  // 8. read data
  eResult = mContext.WaitNoneUpdateAll();
  if( eResult == XN_STATUS_OK )
  {
    // 9a. get the depth map
    const XnDepthPixel*  pDepthMap = mDepthGenerator.GetDepthMap();
    // 9b. get the image map
    const XnUInt8*    pImageMap = mImageGenerator.GetImageMap();
  }
  
  // 10. stop
  mContext.StopGeneratingAll();
  mContext.Shutdown();
 
  return 0;
}
