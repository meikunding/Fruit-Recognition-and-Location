#include "stdafx.h"
#include <cv.h>
#include <highgui.h>
#include <XnCppWrapper.h>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace xn; 
using namespace std; 

// kinect 标定参数
#define CAM_PARAM_FK 524.503
#define CAM_PARAM_FL 524.489
#define CAM_PARAM_U0 322.452
#define CAM_PARAM_V0 256.822

#define K1   0.300702
#define K2   -1.1174
#define P1   -0.00522
#define P2   0.00433
#define K3   1.83874

#define WAIT_KEY_TIME 1			//按键超时时间
#define SHOW_IMAGE_SOURCE 1		//是否显示原图像
#define SHOW_IMAGE_SEGMENT 1	//是否显示通过深度分割后的图像
#define SHOW_IMAGE_CONTOUR 1	//是否显示检测到的轮廓
#define SHOW_IMAGE_THRESH 1		//是滞显示二值化图像
#define PRINT_FRUIT_TOTAL 10		//打印水果总数信息	
#define SHOW_CONFIG_WINDOW 0

#define CENTER_POINTS_TOTAL 1	//平均中心点总数

/*  用于深度分割的深度区间   */
#define DEPTH_LOW 900       
#define DEPTH_HIGH 1500

#define H_THRESHOLD_LOW 0
#define H_THRESHOLD_HIGH 360

#define S_THRESHOLD_LOW 0
#define S_THRESHOLD_HIGH 1.0

#define V_THRESHOLD_LOW 0
#define V_THRESHOLD_HIGH 255

#define DEF_GREEN_PEPPER_H_VAL_LOW	    70 
#define DEF_GREEN_PEPPER_H_VAL_HIGH     135
#define DEF_GREEN_PEPPER_S_VAL_LOW		0.1 
#define DEF_GREEN_PEPPER_S_VAL_HIGH     1.0 
#define DEF_GREEN_PEPPER_V_VAL_LOW		0
#define DEF_GREEN_PEPPER_V_VAL_HIGH		255

#define DEF_APPLE_H_VAL_LOW	    0 
#define DEF_APPLE_H_VAL_HIGH    20
#define DEF_APPLE_S_VAL_LOW		0.5 
#define DEF_APPLE_S_VAL_HIGH    1.0
#define DEF_APPLE_V_VAL_LOW		120
#define DEF_APPLE_V_VAL_HIGH	250

#define DEF_BANANA_H_VAL_LOW	    35 
#define DEF_BANANA_H_VAL_HIGH       60
#define DEF_BANANA_S_VAL_LOW		0.45
#define DEF_BANANA_S_VAL_HIGH       1.0 
#define DEF_BANANA_V_VAL_LOW		0 
#define DEF_BANANA_V_VAL_HIGH		255 

#define DEF_PEAR_H_VAL_LOW	    40 
#define DEF_PEAR_H_VAL_HIGH     65
#define DEF_PEAR_S_VAL_LOW		0.4
#define DEF_PEAR_S_VAL_HIGH     0.6
#define DEF_PEAR_V_VAL_LOW		0 
#define DEF_PEAR_V_VAL_HIGH		255 

#define DEF_ORANGE_H_VAL_LOW	  20 
#define DEF_ORANGE_H_VAL_HIGH     40
#define DEF_ORANGE_S_VAL_LOW	  0.60 
#define DEF_ORANGE_S_VAL_HIGH     1.0 
#define DEF_ORANGE_V_VAL_LOW	  100 
#define DEF_ORANGE_V_VAL_HIGH	  255 

#define DEF_TOMATO_H_VAL_LOW	  210 
#define DEF_TOMATO_H_VAL_HIGH     270
#define DEF_TOMATO_S_VAL_LOW	  0.4
#define DEF_TOMATO_S_VAL_HIGH     1.0 
#define DEF_TOMATO_V_VAL_LOW	  0 
#define DEF_TOMATO_V_VAL_HIGH	  255 

#define COLOR_TEXT   CV_RGB(255,255,255)
#define COLOR_FRAME   CV_RGB(0,0,0)
#define COLOR_LINE   CV_RGB(0,0,0)

#define PI  3.1416

// 水果上方需要增加的冗余量
#define BANANA_Z 80   
#define APPLE_Z 100   
#define PEAR_Z 100  
#define GP_Z  100   
#define ORANGE_Z 100
#define TOMATO_Z 90

// 水果半径大小  Y方向
#define BANANA_Y 27   
#define APPLE_Y 42   
#define PEAR_Y 37  
#define GP_Y  36 
#define ORANGE_Y 40  
#define TOMATO_Y  36 

// 基底与kinect之间的坐标关系
//#define Kinect_BaseX  -86.6
//#define Kinect_BaseY  -1948
//#define Kinect_BaseZ  508

//  画图用的图的大小
#define Thickness 2

// RGB结构体，用于保存有用深度信息内的RGB图像信息
typedef struct 
{
	unsigned char R;
	unsigned char G;
	unsigned char B;
}RGBPIXEL;

// 深度信息为16位
typedef struct 
{
	unsigned char data_L;	//Low bits 
	unsigned char data_H;	//Heigh bits
}DEPTHPIXEL;

typedef enum {
	GreenPepper,	//青椒
	Apple,			//苹果
	Banana,			//香蕉
	Pear,			//梨
	Orange,         //橙子
	Tomato          //西红柿
}FruitKinds;

const CvSize image_size = cvSize(XN_VGA_X_RES, XN_VGA_Y_RES);

float hValLow_GP  = 0;
float hValHigh_GP = 0;
float sValLow_GP  = 0;
float sValHigh_GP = 0;
float vValLow_GP  = 0;
float vValHigh_GP = 0;

// FRUIT_Y水果在Y轴方向上需要增加的量
float FRUIT_Y =0;
// FRUIT_Z水果在Y轴方向上需要增加的量
float FRUIT_Z =0;

//  机械臂与kinect之间的旋转位移矩阵
float RTarray[]={1.0776,  -0.0872,    -0.0435,       -16.4424,
	              0.0283,   0.9825,   0.0477,    -1909.0842,
				  0.02045,  0.0081,   1.101,   517.3334,
                   0,           0,        0,     1};

//水果相对于基地坐标系的坐标
//float Fruit_BaseX=0;
//float Fruit_BaseY=0;
//float Fruit_BaseZ=0;


const char* imgWndName = "Source";
const char* segmentWndName = "Segment";
const char* grayWndName = "Threshold";
const char* contoursWndName = "Contour";
const char* paramConfigWnd = "Param Settings";

const char* trackBarNameHL = "H_Low";
const char* trackBarNameHH = "H_High";
const char* trackBarNameSL = "S_Low";
const char* trackBarNameSH = "S_High";
const char* trackBarNameVL = "V_Low";
const char* trackBarNameVH = "V_High";

int oldPoints = 0;
CvFont font;
vector<CvPoint> vecCenterPoints;
vector<vector<CvPoint3D32f> > vecCenter3DPoints;
//	GreenPepper,	//青椒
//	Apple,			//苹果
//	Banana,			//香蕉
//	Pear			//梨
//	Orange			//橙子
//	Tomato			//西红柿
const FruitKinds RecoFruitKind = FruitKinds::Pear;

//point cloud data struct
struct SColorPoint3D
{
	float  X;
	float  Y;
	float  Z;
	XnUInt8  R;
	XnUInt8  G;
	XnUInt8  B;

	SColorPoint3D( XnPoint3D pos, XnRGB24Pixel color )
	{
		X = pos.X;
		Y = pos.Y;
		Z = pos.Z;
		R = color.nRed;
		G = color.nGreen;
		B = color.nBlue;
	}
};

void imgProc(IplImage* image,IplImage* srcimage, const vector<SColorPoint3D>& vPointCloud);
void GeneratePointCloud( DepthGenerator& rDepthGen,
	const XnDepthPixel* pDepth,
	const XnRGB24Pixel* pImage,
	vector<SColorPoint3D>& vPointCloud );
void checkErrorState(XnStatus status, char* msg);
void Cabliration3D(float x,float y,float z );

char* getFruitName(FruitKinds kind){
	char* name = NULL;
	switch(kind){
	case GreenPepper:
		name = "Green Pepper";
		break;
	case Apple:
		name = "Apple";
		break;
	case Banana:
		name = "Banana";
		break;
	case Pear:
		name = "Pear";
		break;
	case Tomato:
		name = "Tomato";
		break;
	case Orange:
		name = "Orange";
		break;
	default:
		break;
	}
	return name;
}

void point2DTo3D( const vector<SColorPoint3D>& vPointCloud, CvPoint point2d, CvPoint3D32f &point3d){
	_ASSERT(vPointCloud.size() > 0);
	
	int idx = point2d.y*image_size.width + point2d.x;
	point3d.x = vPointCloud[idx].X;
	point3d.y = vPointCloud[idx].Y;
	point3d.z = vPointCloud[idx].Z;
}

void setDefVal(FruitKinds kind){
	char* name = NULL;
	switch(kind){
	case GreenPepper:
		hValLow_GP  = DEF_GREEN_PEPPER_H_VAL_LOW;
		hValHigh_GP = DEF_GREEN_PEPPER_H_VAL_HIGH;
		sValLow_GP  = DEF_GREEN_PEPPER_S_VAL_LOW;
		sValHigh_GP = DEF_GREEN_PEPPER_S_VAL_HIGH;
		vValLow_GP  = DEF_GREEN_PEPPER_V_VAL_LOW;
		vValHigh_GP = DEF_GREEN_PEPPER_V_VAL_HIGH;
		FRUIT_Y = GP_Y;
		FRUIT_Z = GP_Z;
		break;
	case Apple:
		hValLow_GP  = DEF_APPLE_H_VAL_LOW;
		hValHigh_GP = DEF_APPLE_H_VAL_HIGH;
		sValLow_GP  = DEF_APPLE_S_VAL_LOW;
		sValHigh_GP = DEF_APPLE_S_VAL_HIGH;
		vValLow_GP  = DEF_APPLE_V_VAL_LOW;
		vValHigh_GP = DEF_APPLE_V_VAL_HIGH;
		FRUIT_Y = APPLE_Y;
		FRUIT_Z = APPLE_Z;
		break;
	case Banana:
		hValLow_GP  = DEF_BANANA_H_VAL_LOW;
		hValHigh_GP = DEF_BANANA_H_VAL_HIGH;
		sValLow_GP  = DEF_BANANA_S_VAL_LOW;
		sValHigh_GP = DEF_BANANA_S_VAL_HIGH;
		vValLow_GP  = DEF_BANANA_V_VAL_LOW;
		vValHigh_GP = DEF_BANANA_V_VAL_HIGH;
		FRUIT_Y = BANANA_Y;
		FRUIT_Z = BANANA_Z;
		break;
	case Orange:
		hValLow_GP  = DEF_ORANGE_H_VAL_LOW;
		hValHigh_GP = DEF_ORANGE_H_VAL_HIGH;
		sValLow_GP  = DEF_ORANGE_S_VAL_LOW;
		sValHigh_GP = DEF_ORANGE_S_VAL_HIGH;
		vValLow_GP  = DEF_ORANGE_V_VAL_LOW;
		vValHigh_GP = DEF_ORANGE_V_VAL_HIGH;
		FRUIT_Y =ORANGE_Y;
		FRUIT_Z =ORANGE_Z;
		break;
	case Tomato:
		hValLow_GP  = DEF_TOMATO_H_VAL_LOW;
		hValHigh_GP = DEF_TOMATO_H_VAL_HIGH;
		sValLow_GP  = DEF_TOMATO_S_VAL_LOW;
		sValHigh_GP = DEF_TOMATO_S_VAL_HIGH;
		vValLow_GP  = DEF_TOMATO_V_VAL_LOW;
		vValHigh_GP = DEF_TOMATO_V_VAL_HIGH;
		FRUIT_Y = TOMATO_Y;
		FRUIT_Z = TOMATO_Z;
		break;
	case Pear:
		hValLow_GP  = DEF_PEAR_H_VAL_LOW;
		hValHigh_GP = DEF_PEAR_H_VAL_HIGH;
		sValLow_GP  = DEF_PEAR_S_VAL_LOW;
		sValHigh_GP = DEF_PEAR_S_VAL_HIGH;
		vValLow_GP  = DEF_PEAR_V_VAL_LOW;
		vValHigh_GP = DEF_PEAR_V_VAL_HIGH;
		FRUIT_Y =PEAR_Y;
		FRUIT_Z =PEAR_Z;
		break;
	default:
		break;
	}
	
}

void setHChannelLow(int val){
	hValLow_GP = (float)val;
}
void setHChannelHigh(int val){
	hValHigh_GP = (float)val;
}

void setSChannelLow(int val){
	sValLow_GP = val/(float)100;;
}

void setSChannelHigh(int val){
	sValHigh_GP =  val/(float)100;;
}

void setVChannelLow(int val){
	vValLow_GP = (float)val;
}

void setVChannelHigh(int val){
	vValHigh_GP = (float)val;
}

void initTrackBarWindow()
{
	cvNamedWindow(paramConfigWnd, CV_WINDOW_NORMAL);

	cvCreateTrackbar(trackBarNameHL, paramConfigWnd, H_THRESHOLD_LOW, H_THRESHOLD_HIGH, setHChannelLow);
	cvCreateTrackbar(trackBarNameHH, paramConfigWnd, H_THRESHOLD_LOW, H_THRESHOLD_HIGH, setHChannelHigh);
	cvCreateTrackbar(trackBarNameSL, paramConfigWnd, S_THRESHOLD_LOW, S_THRESHOLD_HIGH*100, setSChannelLow);
	cvCreateTrackbar(trackBarNameSH, paramConfigWnd, S_THRESHOLD_LOW, S_THRESHOLD_HIGH*100, setSChannelHigh);
	cvCreateTrackbar(trackBarNameVL, paramConfigWnd, V_THRESHOLD_LOW, V_THRESHOLD_HIGH, setVChannelLow);
	cvCreateTrackbar(trackBarNameVH, paramConfigWnd, V_THRESHOLD_LOW, V_THRESHOLD_HIGH, setVChannelHigh);
	cvSetTrackbarPos(trackBarNameHL, paramConfigWnd, hValLow_GP);
	cvSetTrackbarPos(trackBarNameHH, paramConfigWnd, hValHigh_GP);
	cvSetTrackbarPos(trackBarNameSL, paramConfigWnd, sValLow_GP*100);
	cvSetTrackbarPos(trackBarNameSH, paramConfigWnd, sValHigh_GP*100);
	cvSetTrackbarPos(trackBarNameVL, paramConfigWnd, vValLow_GP);
	cvSetTrackbarPos(trackBarNameVH, paramConfigWnd, vValHigh_GP); 
}

void findCenter3D(int FruitNum, const vector<SColorPoint3D>& vPointCloud)
{
	if (FruitNum > 0)
	{
		if (oldPoints != FruitNum)
		{
			for (int i = 0; i < vecCenter3DPoints.size(); i++)
			{
				vecCenter3DPoints[i].clear();
			}
			vecCenter3DPoints.clear();
		}

		if (vecCenter3DPoints.size() == 0){
			vecCenter3DPoints.resize(FruitNum);
		}

		CvPoint3D32f point3d;
		CvPoint3D32f cabpoint3D;
		for (int i = 0; i < vecCenter3DPoints.size(); i++)
		{
			point2DTo3D(vPointCloud, vecCenterPoints[i], point3d);
			cabpoint3D.x=((vecCenterPoints[i].x-CAM_PARAM_U0)*point3d.z)/CAM_PARAM_FK;
			cabpoint3D.y=((vecCenterPoints[i].y-CAM_PARAM_V0)*point3d.z)/CAM_PARAM_FL;
			cabpoint3D.z=point3d.z;
			cout<<"Calibration :"<<" X: "<<cabpoint3D.x<<" Y: "<<cabpoint3D.y<<" Z:"<<point3d.z<<endl;
			if(point3d.x == 0 && point3d.y == 0 && point3d.z == 0)
			{
				continue;
				//cout << "\nFind invalid point."<<endl;	
			}
			else
			{
				vecCenter3DPoints[i].push_back(point3d);
			}
		}

	}
	oldPoints = FruitNum;
	if(vecCenter3DPoints.size()>0)
	{
		for (int i = 0; i < vecCenter3DPoints.size(); i++)
		{

			if (vecCenter3DPoints[i].size() == CENTER_POINTS_TOTAL)
			{
#if 0
				//PRINT_FRUIT_TOTAL
				if (m == 1)
					cout << "Find "<< m << " "<< getFruitName(RecoFruitKind) << endl;
				else
					cout << "Find "<< m << " "<< getFruitName(RecoFruitKind) << "s" << endl;
#endif
				float avg_x = 0, avg_y = 0, avg_z = 0;
				for (int j = 0; j < CENTER_POINTS_TOTAL; j++)
				{
					avg_x += vecCenter3DPoints[i][j].x;
					avg_y += vecCenter3DPoints[i][j].y;
					avg_z += vecCenter3DPoints[i][j].z;
				}
				avg_x /= CENTER_POINTS_TOTAL;
				avg_y /= CENTER_POINTS_TOTAL;
				avg_z /= CENTER_POINTS_TOTAL;
                cout<<"Kinect ： "<< getFruitName(RecoFruitKind) << i+1 <<"：x：" << avg_x << ", y：" << avg_y<<", z："<< avg_z << endl;
				//float cab_x=CAM_PARAM_FK*(avg_x/avg_z)+CAM_PARAM_U0;
				//float cab_y=CAM_PARAM_FL*(avg_y/avg_z)+CAM_PARAM_V0;
				//CvPoint cab;
				//cab.x=cab_x;
				//cab.y=cab_y;
				//CvPoint3D32f cab3D;
				//point2DTo3D(vPointCloud, cab, cab3D);
				//cout<<"Calibration :"<<" X: "<<cab3D.x<<" Y: "<<cab3D.y<<" Z:"<<cab3D.z<<endl;
				
		       /* Cabliration3D(avg_x,avg_y,avg_z);*/


				// 标定时使用
				//avg_z=avg_z+8;
				//cout.precision(1);
				//cout.setf(ios::fixed);
				//cout<<"Kinect ： "<< getFruitName(RecoFruitKind) << i+1 <<"：x：" << avg_x << ", y：" << avg_y<<", z："<< avg_z << endl;

				//// 利用RT矩阵乘上kinect坐标点矩阵得到base下的坐标
				//avg_z=avg_z+FRUIT_Y;
				//float Fruit3Dpoint[]={avg_x,avg_z,avg_y,1};

				//CvMat *RTMatrix=cvCreateMat(4,4,CV_32FC1);
				//CvMat *PointMatrix=cvCreateMat(4,1,CV_32FC1);
				//CvMat *ResultMatrix=cvCreateMat(4,1,CV_32FC1);

				//cvSetData(PointMatrix,Fruit3Dpoint,PointMatrix->step);
				//cvSetData(RTMatrix,RTarray,RTMatrix->step);
				//cvmMul(RTMatrix,PointMatrix,ResultMatrix);

				//float Fruit_BaseX =  CV_MAT_ELEM(*ResultMatrix,float,0,0)/10;
				//float Fruit_BaseY =  CV_MAT_ELEM(*ResultMatrix,float,1,0)/10;
				//float Fruit_BaseZ =  CV_MAT_ELEM(*ResultMatrix,float,2,0)/10;
				//cout <<"Base  "<< getFruitName(RecoFruitKind) << i+1 <<"：x：" << Fruit_BaseX << ", y：" << Fruit_BaseY <<", z："<< Fruit_BaseZ << endl;
				//cvReleaseMat(&RTMatrix);
				//cvReleaseMat(&PointMatrix);
				//cvReleaseMat(&ResultMatrix);
			    

				vecCenter3DPoints[i].clear();
				if(i == vecCenter3DPoints.size())cout << endl;
			}
		}

	}
}
void Cabliration3D(float x,float y,float z)
{
    float x1=x/z;
	float y1=y/z;
	float r=sqrt(x1*x1+y1*y1);
	float x2=x1*(1+K1*r*r+K2*r*r*r*r)+2*P1*x1*y1+P2*(r*r+2*x1*x1);
	float y2=y1*(1+K1*r*r+K2*r*r*r*r)+2*P2*x1*y1+P1*(r*r+2*y1*y1);
	float u =CAM_PARAM_FK*x2+CAM_PARAM_U0;
	float v =CAM_PARAM_FL*y2+CAM_PARAM_V0;
	cout<<"Calibration ： "<< getFruitName(RecoFruitKind) <<"：x：" << u << ", y：" << v<< endl;
}
void DrawBox(CvBox2D box,IplImage *segmentimage,IplImage *srcimage)
{
	double angle = box.angle*PI/180;
	float a = (float)cos(angle)*0.5f; 
	float b = (float)sin(angle)*0.5f; 
	CvPoint2D32f  point[4];
	for (int i=0; i<4; i++) 
	{ 
		point[i].x = 0; 
		point[i].y = 0; 
	} 
	cvBoxPoints(box, point);
	CvPoint pt[4]; 
	for ( int i=0; i<4; i++) 
	{ 
		pt[i].x = (int)point[i].x; 
		pt[i].y = (int)point[i].y; 
	} 
	cvLine( srcimage, pt[0], pt[1],COLOR_FRAME, Thickness, 8, 0 ); 
	cvLine( srcimage, pt[1], pt[2],COLOR_FRAME, Thickness, 8, 0 ); 
	cvLine( srcimage, pt[2], pt[3],COLOR_FRAME, Thickness, 8, 0 ); 
	cvLine( srcimage, pt[3], pt[0],COLOR_FRAME, Thickness, 8, 0 );

	cvLine( segmentimage, pt[0], pt[1],COLOR_FRAME, Thickness, 8, 0 ); 
	cvLine( segmentimage, pt[1], pt[2],COLOR_FRAME, Thickness, 8, 0 ); 
	cvLine( segmentimage, pt[2], pt[3],COLOR_FRAME, Thickness, 8, 0 ); 
	cvLine( segmentimage, pt[3], pt[0],COLOR_FRAME, Thickness, 8, 0 );

}


/* 利用深度信息分割图像 , 得到有用的图像 */
void GetROIdata(IplImage* srcimage,IplImage* depthimage,IplImage* dstimage)
{
	_ASSERT(srcimage);
	_ASSERT(depthimage->nChannels==1);
	_ASSERT(DEPTH_LOW>0&&DEPTH_HIGH>DEPTH_LOW);
	const ushort *pdepthdata=(ushort*)depthimage->imageData;
	const uchar *psrcdata=(uchar*)srcimage->imageData;
	uchar *pdstdata=(uchar*)dstimage->imageData;
	for(int j = 0;j < image_size.height;j++)//We just want to get depth datas rang from 1000mm to 1500mm
	{

		for(int i = 0;i < image_size.width;i++)
		{
			int index=j*image_size.width;
			int index2=j*(dstimage->widthStep);
			if (pdepthdata[index+i]<DEPTH_HIGH&&pdepthdata[index+i]>DEPTH_LOW)
			{
				pdstdata[index2+3*i]=psrcdata[index2+3*i];
				pdstdata[index2+3*i+1]=psrcdata[index2+3*i+1];
				pdstdata[index2+3*i+2]=psrcdata[index2+3*i+2];
			}
			else
			{
				pdstdata[index2+3*i]= 0;
				pdstdata[index2+3*i+1]=0;
				pdstdata[index2+3*i+2]=0;
			
			}


		}
	}

}

int main()
{
	cout << "Recognize fruit: " << getFruitName(RecoFruitKind) << endl;
	XnStatus eResult = XN_STATUS_OK;
	 //init
	Context mContext;
	eResult = mContext.Init(); 
	checkErrorState(eResult,"Init Context failed\n");

	DepthGenerator mDepthGenerator;
	eResult = mDepthGenerator.Create(mContext);
	checkErrorState(eResult,"Create DepthGenerator failed\n");

	ImageGenerator mImageGenerator;
	eResult = mImageGenerator.Create(mContext);
	checkErrorState(eResult,"Create ImageGenerator failed\n");

	 //set output mode
	XnMapOutputMode mapMode;
	mapMode.nXRes = image_size.width;  // 640
	mapMode.nYRes = image_size.height;  // 480
	mapMode.nFPS  = 30;
	eResult = mDepthGenerator.SetMapOutputMode(mapMode);
	checkErrorState(eResult,"DepthGenerator SetMapOutputMode failed\n");

	eResult = mImageGenerator.SetMapOutputMode(mapMode);
	checkErrorState(eResult,"ImageGenerator SetMapOutputMode failed\n");

	mDepthGenerator.GetAlternativeViewPointCap().SetViewPoint( mImageGenerator );
	//start generating  
	eResult = mContext.StartGeneratingAll();  

	 //read data
	vector<SColorPoint3D> vPointCloud;
	
	//OpenCV  
	IplImage* imgRGB8u=cvCreateImage(image_size,IPL_DEPTH_8U,3);  
	IplImage* imgBGR8u=cvCreateImage(image_size,IPL_DEPTH_8U,3); 
	IplImage* depthImg = cvCreateImage(image_size,IPL_DEPTH_16U,1);
	setDefVal(RecoFruitKind);

#if SHOW_CONFIG_WINDOW
	initTrackBarWindow();
#endif
	cvInitFont(&font, CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1.0, 1.0);
	double time = 0.0;
	double freq = cvGetTickFrequency()*1000;
	while ( !xnOSWasKeyboardHit() )
	{
		eResult = mContext.WaitNoneUpdateAll();
		 //get the depth map
		const XnDepthPixel*  pDepthMap = mDepthGenerator.GetDepthMap();

		 //get the image map
		const XnRGB24Pixel*  pImageMap = mImageGenerator.GetRGB24ImageMap();
   
		 //generate point cloud
		vPointCloud.clear();
        //double t = (double)cvGetTickCount();
		GeneratePointCloud(mDepthGenerator, pDepthMap, pImageMap, vPointCloud );
		memcpy(imgRGB8u->imageData,pImageMap,image_size.width*image_size.height*3);  
		memcpy(depthImg->imageData,pDepthMap,image_size.width*image_size.height*2);
		
		cvCvtColor(imgRGB8u,imgBGR8u,CV_RGB2BGR);
		imgProc(imgBGR8u,depthImg,vPointCloud);
		//findCenter3D(vPointCloud);
	 //   t = (double)cvGetTickCount() - t;
		//cout<<"exec time"<<t/(cvGetTickFrequency()*1e6)<<endl;
	}
	cvReleaseImage(&imgRGB8u);  
	cvReleaseImage(&imgBGR8u);
	cvReleaseImage(&depthImg);

	//stop
	mContext.StopGeneratingAll();  
	mContext.Shutdown();  

	return 0;
}

void filter(IplImage *_src, IplImage **dst){
	IplImage *src = cvCloneImage(_src);

	if(*dst)
	{
		cvReleaseImage(&(*dst));
		*dst = NULL;
	}
	*dst = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);

	float data[9]={
		0, -1, 0,
		-1, 5, -1,
		0, -1, 0
				};
	CvMat kernel = cvMat(3, 3, CV_32F, data);
	cvFilter2D(src, *dst, &kernel);
	cvReleaseImage(&src);
}

void drawHist(IplImage *input, IplImage **output, int bins, float scale = 2)
{
	_ASSERT(input->nChannels == 1);
	_ASSERT(bins > 1);
	float range[] = {0, bins-1};
	float *ranges[] = {range};
	*output = cvCreateImage(cvSize(bins*scale, bins), input->depth, 1);
	cvZero(*output);

	CvHistogram *hist = cvCreateHist(1, &bins, CV_HIST_ARRAY, ranges);
	cvCalcHist(&input, hist);
	cvNormalizeHist(hist,1.0);
	float max_value;
	cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );
	
	for (int i = 0; i < bins; i++)
	{
		float bin_val = cvQueryHistValue_1D(hist,i);
		int intensity = cvRound(bin_val*bins/max_value);
		CvPoint pt1, pt2;
		pt1.x = i*scale;
		pt1.y = (*output)->height-intensity;
		pt2.x = i*scale;
		pt2.y = (*output)->height;
		cvLine(*output, pt1, pt2, cvScalar(255),scale, CV_FILLED);
	}
	
	cvReleaseHist(&hist);
}

void imgProc(IplImage* srcimage,IplImage* depthimage,const vector<SColorPoint3D> &vPointCloud)
{
	vector<RGBPIXEL> ROIimage;
	vector<DEPTHPIXEL> ROIdepth;
	vector<CvPoint> ROIimagePoint;
	IplImage *srcimageClone = cvCloneImage(srcimage);
	IplImage *depthClone =  cvCloneImage(depthimage);
	IplImage* roiimage=cvCreateImage(image_size,IPL_DEPTH_8U,3);
	cvZero(roiimage);
	ROIimage.clear();
	ROIdepth.clear();
	ROIimagePoint.clear();
	//double t=(double)cvGetTickCount();
	GetROIdata(srcimageClone,depthClone,roiimage);
	//t = (double)cvGetTickCount() - t;
	//cout<<"exec time"<<t/(cvGetTickFrequency()*1e6)<<endl;
	IplImage *imageClone = cvCreateImage(image_size,IPL_DEPTH_8U,3);
	cvZero(imageClone);
	cvCopy(roiimage,imageClone);
	//cvSaveImage("segmentimage.jpg",imageClone);
	IplImage *image32F	 = cvCreateImage(image_size, IPL_DEPTH_32F, 3);
	IplImage *imageHSV	 = cvCreateImage(image_size, IPL_DEPTH_32F, 3);
	IplImage *hImg		 = cvCreateImage(image_size, IPL_DEPTH_32F, 1);
	IplImage *sImg		 = cvCreateImage(image_size, IPL_DEPTH_32F, 1);
	IplImage *vImg		 = cvCreateImage(image_size, IPL_DEPTH_32F, 1);
	IplImage *gray		 = cvCreateImage(image_size, IPL_DEPTH_8U,  1);
	IplImage *edge		 = cvCreateImage(image_size, IPL_DEPTH_8U,  1);
	CvMemStorage *storage = cvCreateMemStorage(0);
	cvZero(edge);
	cvConvertScale(imageClone,image32F);

	cvCvtColor(image32F,imageHSV,CV_BGR2HSV);
	cvSplit(imageHSV,hImg,sImg,vImg,NULL);

#if 0 
	IplImage *hist_img;
	drawHist(hImg,&hist_img, 360);
	cvShowImage("Hist", hist_img);
	cvReleaseImage(&hist_img);
#endif
	
	double hVal = 0.0,sVal = 0.0,vVal = 0.0;
	double pixel = 0.0;
	int index = 0;
	
	char* grayData = gray->imageData;
	char* edgeData = edge->imageData;
	float* hImgData = (float*)hImg->imageData;
	float* sImgData = (float*)sImg->imageData;
	float* vImgData = (float*)vImg->imageData;

	switch(RecoFruitKind){
	case FruitKinds::GreenPepper:
			for (int row = 0 ; row < image_size.height; row++)
			{
				for (int col = 0 ; col < image_size.width; col++)
				{
					index = row*gray->width + col;
					hVal = hImgData[index];
					sVal = sImgData[index];
					vVal = vImgData[index];

					if(hValLow_GP<=hVal&&hVal<=hValHigh_GP){
						if(sValLow_GP<=sVal&&sVal<=sValHigh_GP){
							if(vValLow_GP<=vVal&&vVal<=vValHigh_GP){
								pixel=255;
							}
						}
					}
					else{
						pixel=0;
					}

					grayData[index] = pixel;
				}

			}
		
		break;

	case FruitKinds::Apple:
		for (int row = 0 ; row < image_size.height; row++)
		{
			for (int col = 0 ; col < image_size.width; col++)
			{
				index = row*gray->width + col;
				hVal = hImgData[index];
				sVal = sImgData[index];
				vVal = vImgData[index];

				if((hValLow_GP<=hVal&&hVal<=hValHigh_GP)||((360 - hValHigh_GP)<=hVal&&hVal<=(360 - hValLow_GP)))
				{
					if(sValLow_GP<=sVal&&sVal<=sValHigh_GP){
						if(vValLow_GP<=vVal&&vVal<=vValHigh_GP){
							pixel=255;
						}
					}
				}
				else{
					pixel=0;
				}

				grayData[index] = pixel;

			}
		}
		
		break;

	case FruitKinds::Banana:
		for (int row = 0 ; row < image_size.height; row++)
		{
			for (int col = 0 ; col < image_size.width; col++)
			{
				index = row*gray->width + col;
				hVal = hImgData[index];
				sVal = sImgData[index];
				vVal = vImgData[index];

				if(hValLow_GP<=hVal&&hVal<=hValHigh_GP){
					if(sValLow_GP<=sVal&&sVal<=sValHigh_GP){
						if(vValLow_GP<=vVal&&vVal<=vValHigh_GP){
							pixel=255;
						}
					}
				}
				else{
					pixel=0;
				}

				grayData[index] = pixel;
			}
		}
		
		break;

	case FruitKinds::Pear:
		for (int row = 0 ; row < image_size.height; row++)
		{
			for (int col = 0 ; col < image_size.width; col++)
			{
				index = row*gray->width + col;
				hVal = hImgData[index];
				sVal = sImgData[index];
				vVal = vImgData[index];

				if(hValLow_GP<=hVal&&hVal<=hValHigh_GP){
					if(sValLow_GP<=sVal&&sVal<=sValHigh_GP){
						if(vValLow_GP<=vVal&&vVal<=vValHigh_GP){
							pixel=255;
						}
					}
				}
				else{
					pixel = 0;
				}

				grayData[index] = pixel;
			}
		}
		
		break;
	case FruitKinds::Orange:
		for (int row = 0 ; row < image_size.height; row++)
		{
			for (int col = 0 ; col < image_size.width; col++)
			{
				index = row*gray->width + col;
				hVal = hImgData[index];
				sVal = sImgData[index];
				vVal = vImgData[index];

				if(hValLow_GP<=hVal&&hVal<=hValHigh_GP){
					if(sValLow_GP<=sVal&&sVal<=sValHigh_GP){
						if(vValLow_GP<=vVal&&vVal<=vValHigh_GP){
							pixel=255;
						}
					}
				}
				else{
					pixel = 0;
				}

				grayData[index] = pixel;
			}
		}

		break;
	case FruitKinds::Tomato:
		for (int row = 0 ; row < image_size.height; row++)
		{
			for (int col = 0 ; col < image_size.width; col++)
			{
				index = row*gray->width + col;
				hVal = hImgData[index];
				sVal = sImgData[index];
				vVal = vImgData[index];

				if((hValLow_GP<=hVal&&hVal<=hValHigh_GP)||(360 - hValHigh_GP<=hVal&&hVal<=(360 - hValLow_GP)/2)){
					if(sValLow_GP<=sVal&&sVal<=sValHigh_GP){
						if(vValLow_GP<=vVal&&vVal<=vValHigh_GP){
							pixel=255;
						}
					}
				}
				else{
					pixel = 0;
				}

				grayData[index] = pixel;
			}
		}

		break;

	default:
		break;
	}

	cvDilate(gray, gray);
	cvErode(gray, gray);
	cvErode(gray, gray);
	cvDilate(gray, gray);

	IplImage *grayEnhance = cvCloneImage(gray);
	CvSeq *contours = NULL;
	int nContours = cvFindContours(grayEnhance,storage,&contours,sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	//cout << nContours << endl;
	cvDrawContours(edge, contours, cvScalar(255),cvScalar(255), 1);
	int Fruitcount=0;
	bool isSkip = false;
	vecCenterPoints.clear();
	float intrinsic_array[]={524.503,0,322.452,0,524.489,256.822,0,0,1};
	float distort_array[]={0.322854,-1.4575,-0.00216,0.00184};
	CvMat *intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *distort_matrix=cvCreateMat(4,1,CV_32FC1);
	for (;contours != NULL; contours = contours->h_next)
	{
		//cout <<contours->total << endl;
		if (contours->total < 50 || contours->total > 700)
		{
			isSkip = true;
		}
		else
		{
			CvPoint _center;
			double area = cvContourArea(contours, CV_WHOLE_SEQ);
			/*cout<<"area :  "<<area<<endl;*/
			double length =cvArcLength(contours,CV_WHOLE_SEQ);
			double roundper;
			roundper=4*3.14*fabs(area)/(fabs(length)*fabs(length));
			printf("rounder:%lf\n",roundper);
			CvBox2D box = cvMinAreaRect2(contours, storage);
			int width = max(box.size.width,box.size.height);
			int height = min(box.size.width,box.size.height);
			double h_w_ratio = height / (double)width;
			//cout<<"h_w_ratio : "<<h_w_ratio<<endl;
			_center.x = (int)box.center.x;
			_center.y = (int)box.center.y;
			if (area > 300)	
			{
				
				if (RecoFruitKind == GreenPepper || RecoFruitKind == Apple || RecoFruitKind == Pear||RecoFruitKind == Orange
					||RecoFruitKind == Tomato)
				{
					if (0.6 <= h_w_ratio && h_w_ratio <= 1.0)	//宽高比
					{
						/*DrawBox(box,imageClone,srcimageClone);*/
						isSkip = false;			
						Fruitcount++;
						/*cout<<"Before Calibration"<<"X= "<<_center.x<<"  Y= "<<_center.y<<endl;*/
						//CvMat *centermatrix=cvCreateMat(2,1,CV_8U);
						//float centerarray[]={_center.x,_center.y};
						//cvSetData(centermatrix,centerarray,centermatrix->step);
						//CvMat *centermatrix1=cvCreateMat(2,1,CV_8U);
						//cvUndistortPoints(centermatrix,centermatrix1,intrinsic_matrix,distort_matrix,0,0);
						//center.x=CV_MAT_ELEM(*centermatrix1,int,0,0);
						//center.y=CV_MAT_ELEM(*centermatrix1,int,0,1);
						/*cout<<"After Calibration"<<"X= "<<center.x<<"Y= "<<center.y<<endl;*/
						vecCenterPoints.push_back(_center);
					}
					else if (area>2000)
					{
						DrawBox(box,imageClone,srcimage);
						isSkip = false;	
						CvPoint _center1,_center2;
						_center1.x=_center.x+width/4;
						_center1.y=_center.y;
						Fruitcount++;
						vecCenterPoints.push_back(_center1);
						_center2.x=_center.x-width/4;
						_center2.y=_center.y;
						Fruitcount++;
						vecCenterPoints.push_back(_center2);
					}
					else{
						isSkip = true;
					}
				}
				else if (RecoFruitKind == Banana){
					if (area<1000&&0.2<= h_w_ratio && h_w_ratio <= 0.3)	//宽高比
					{

						/*DrawBox(box,imageClone,srcimageClone);*/
						isSkip = false;
						Fruitcount++;
						vecCenterPoints.push_back(_center);
					}
					else if (area>1200)
					{
						DrawBox(box,imageClone,srcimageClone);
						isSkip = false;	
						CvPoint _center1,_center2;
						_center1.x=_center.x+width/4;
						_center1.y=_center.y;
						Fruitcount++;
						vecCenterPoints.push_back(_center1);
						_center2.x=_center.x-width/4;
						_center2.y=_center.y;
						Fruitcount++;
						vecCenterPoints.push_back(_center2);

					}
					else{
						isSkip = true;
					}
				}	
			}
			else{
				isSkip = true;
			}

			if (!isSkip)
			{
				if (area<1000)
				{
				 CvPoint start1, start2, end1, end2;
				 start1.x = 0;
				 start1.y = _center.y;
				 end1.x = image_size.width -1;
				 end1.y = _center.y;

				 start2.x = _center.x;
				 start2.y = 0;
				 end2.x = _center.x;
				 end2.y = image_size.height-1;
				 /* 原图像化出物体的位置 */
				 //cvLine(srcimageClone, start1, end1, CV_RGB(255,0,0));
				 //cvLine(srcimageClone, start2, end2,  CV_RGB(255,0,0));
				 ///* 分割后的图像化出物体的位置 */
				 //cvLine(imageClone, start1, end1, CV_RGB(255,0,0));
				 //cvLine(imageClone, start2, end2,  CV_RGB(255,0,0));
				  /* 轮廓图像化出物体的位置 */
				  //cvLine(edge, start1, end1, cvScalar(255));
				  //cvLine(edge, start2, end2, cvScalar(255));
				}
				else 
				{
					CvPoint start1, start2, end1, end2,start3,end3;
					start1.x = 0;
					start1.y = _center.y;
					end1.x = image_size.width -1;
					end1.y = _center.y;

					start2.x = _center.x+width/4;
					start2.y = 0;
					end2.x = _center.x+width/4;
					end2.y = image_size.height-1;

					start3.x = _center.x-width/4;
					start3.y = 0;
					end3.x = _center.x-width/4;
					end3.y = image_size.height-1;


					//cvLine(imageClone, start1, end1, CV_RGB(255,0,0));
					//cvLine(imageClone, start2, end2,  CV_RGB(255,0,0));
					//cvLine(imageClone, start3, end3,  CV_RGB(255,0,0));

					//cvLine(srcimageClone, start1, end1, CV_RGB(255,0,0));
					//cvLine(srcimageClone, start2, end2,  CV_RGB(255,0,0));
					//cvLine(srcimageClone, start3, end3,  CV_RGB(255,0,0));

					////cvLine(edge, start1, end1, cvScalar(255));
					////cvLine(edge, start2, end2, cvScalar(255));
					////cvLine(edge, start3, end3,  cvScalar(255));

				}
				char text[3];
				for (int i = 0; i < vecCenterPoints.size(); i++)
				{
					sprintf_s(text, "%d", i+1);
					CvPoint org;
					org.x = vecCenterPoints[i].x;
					if (vecCenterPoints[i].y - 15 >= 0)
					{
						org.y = vecCenterPoints[i].y - 10;
					}
					else{
						org.y = vecCenterPoints[i].y + 10;
					}
					//cvPutText(imageClone, text, org, &font, COLOR_TEXT);
					//cvPutText(srcimageClone, text, org, &font, COLOR_TEXT);
					
				}

			}			
		}

		if (isSkip)
		{
			for (int i = 0; i < contours->total; i++)
			{
				CvPoint *pt = (CvPoint *)CV_GET_SEQ_ELEM(CvPoint, contours, i);
				int x = pt->x;
				int y = pt->y;

				index = y*edge->width + x;
				edgeData[index] = 0;
			}
		}
	}
    findCenter3D(Fruitcount,vPointCloud);

#if SHOW_IMAGE_SOURCE
	cvShowImage(imgWndName, srcimageClone);
#endif
#if SHOW_IMAGE_SEGMENT
	cvShowImage(segmentWndName,imageClone);
#endif
#if SHOW_IMAGE_CONTOUR
	cvShowImage(contoursWndName, edge);
#endif
#if SHOW_IMAGE_THRESH
	cvShowImage(grayWndName, gray);
#endif

	cvWaitKey(WAIT_KEY_TIME);
	cvReleaseImage(&srcimageClone);
	cvReleaseImage(&depthClone);
	cvReleaseImage(&roiimage);
	cvReleaseImage(&imageClone);
	cvReleaseImage(&image32F);
	cvReleaseImage(&imageHSV);
	cvReleaseImage(&hImg);
	cvReleaseImage(&sImg);
	cvReleaseImage(&vImg);
	cvReleaseImage(&edge);
	cvReleaseImage(&gray);
	cvReleaseImage(&grayEnhance);
	cvReleaseMemStorage(&storage);
}

void GeneratePointCloud( DepthGenerator& rDepthGen,
	const XnDepthPixel* pDepth,
	const XnRGB24Pixel* pImage,
	vector<SColorPoint3D>& vPointCloud )
{
	// number of point is the number of 2D image pixel
	DepthMetaData mDepthMD;
	rDepthGen.GetMetaData( mDepthMD );
	unsigned int uPointNum = mDepthMD.FullXRes() * mDepthMD.FullYRes();

	// build the data structure for convert
	XnPoint3D* pDepthPointSet = new XnPoint3D[ uPointNum ];
	unsigned int i, j, idxShift, idx;
	for( j = 0; j < mDepthMD.FullYRes(); ++j )
	{
		idxShift = j * mDepthMD.FullXRes();
		for( i = 0; i < mDepthMD.FullXRes(); ++i )
		{
			idx = idxShift + i;
			pDepthPointSet[idx].X = i;
			pDepthPointSet[idx].Y = j;


			pDepthPointSet[idx].Z = pDepth[idx];
		}
	}

	// un-project points to real world
	XnPoint3D* p3DPointSet = new XnPoint3D[ uPointNum ];
	rDepthGen.ConvertProjectiveToRealWorld( uPointNum, pDepthPointSet, p3DPointSet );
	delete[] pDepthPointSet;

	// build point cloud
	for( i = 0; i < uPointNum; ++ i )
	{
		vPointCloud.push_back( SColorPoint3D( p3DPointSet[i], pImage[i] ) );
	}
	delete[] p3DPointSet;
}

void checkErrorState(XnStatus status, char* msg){
	if(status!=XN_STATUS_OK){
		printf(msg);
		exit(-1);
	}

}

