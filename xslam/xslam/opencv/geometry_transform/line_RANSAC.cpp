#include "xslam/opencv/geometry_transform/line_RANSAC.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <time.h>
#include <math.h>

namespace xslam {
namespace opencv {
namespace geometry_transform {


void LineRANSAC::RunDemo()
{
	srand(time(NULL)) ;

	//-------------------------------------------------------------- make sample data  
	/* random number in range 0 - 1 not including 1 */
	float random = 0.f;
	/* the white noise */
	float noise = 0.f;
	/* Setup constants */
	const static int q = 15;
	const static float c1 = (1 << q) - 1;
	const static float c2 = ((int)(c1 / 3)) + 1;
	const static float c3 = 1.f / c1;

	double noise_sigma = 100 ;
	double x[100] ;
	double y[100] ;

	double iA = 0.005 ;
	double iB = 0.5 ;
	double iC = 0 ;
	for( int i=0 ; i<100 ; i++ )
	{
		x[i] = i ;
		y[i] = iA*(x[i]*x[i]) + iB*x[i] + iC ;
		    
		random = ((float)rand() / (float)(RAND_MAX + 1));
		noise = (2.f * ((random * c2) + (random * c2) + (random * c2)) - 3.f * (c2 - 1.f)) * c3;

		int noise_scale = 2.0 ;
		if( i > 50 && i<70 ) noise_scale = 5.0 ;
		y[i] = y[i] + noise*noise_scale ;
	}


	//-------------------------------------------------------------- build matrix
	cv::Mat A(100, 3, CV_64FC1) ;
	cv::Mat B(100,1, CV_64FC1) ;

	for( int i=0 ; i<100 ; i++ )
	{
		A.at<double>(i,0) = x[i] * x[i] ;
	}
	for( int i=0 ; i<100 ; i++ )
	{
		A.at<double>(i,1) = x[i] ;
	}
	for( int i=0 ; i<100 ; i++ )
	{
		A.at<double>(i,2) = 1.0 ;
	}

	for( int i=0 ; i<100 ; i++ )
	{
		B.at<double>(i,0) = y[i] ;
	}

	//-------------------------------------------------------------- RANSAC fitting 
	int n_data = 100 ;
	int N = 100;	//iterations 
	double T = 3*noise_sigma;   // residual threshold

	int n_sample = 3;
	int max_cnt = 0;
	cv::Mat best_model(3,1,CV_64FC1) ;

	for( int i=0 ; i<N ; i++ )
	{
		//random sampling - 3 point  
		int k[3] = {-1, } ;
		k[0] = floor((rand()%100+1))+1;
  
		do
		{
			k[1] = floor((rand()%100+1))+1;
		}while(k[1]==k[0] || k[1]<0) ;
	
		do
		{
			k[2] = floor((rand()%100+1))+1;
		}while(k[2]==k[0] || k[2]==k[1] || k[2]<0) ;

		printf("random sample : %d %d %d\n", k[0], k[1], k[2]) ;

		//model estimation
		cv::Mat AA(3,3,CV_64FC1) ;
		cv::Mat BB(3,1, CV_64FC1) ;
		for( int j=0 ; j<3 ; j++ )
		{
			AA.at<double>(j,0) = x[k[j]] * x[k[j]] ;
			AA.at<double>(j,1) = x[k[j]] ;
			AA.at<double>(j,2) = 1.0 ;
			
			BB.at<double>(j,0) = y[k[j]] ;
		}

		cv::Mat AA_pinv(3,3,CV_64FC1) ;
		invert(AA, AA_pinv, cv::DECOMP_SVD);

		cv::Mat X = AA_pinv * BB ;	

		//evaluation 
		cv::Mat residual(100,1,CV_64FC1) ;
		residual = cv::abs(B-A*X) ;
		int cnt = 0 ;
		for( int j=0 ; j<100 ; j++ )
		{
			double data = residual.at<double>(j,0) ;
			
			if( data < T ) 
			{
				cnt++ ;
			}
		}

		if( cnt > max_cnt ) 
		{
			best_model = X ;
			max_cnt = cnt ;
		}
	}

	//------------------------------------------------------------------- optional LS fitting 
	cv::Mat residual = cv::abs(A*best_model - B) ;
	std::vector<int> vec_index ;
	for( int i=0 ; i<100 ; i++ )
	{
		double data = residual.at<double>(i, 0) ;
		if( data < T ) 
		{
			vec_index.push_back(i) ;
		}
	}

	cv::Mat A2(vec_index.size(),3, CV_64FC1) ;
	cv::Mat B2(vec_index.size(),1, CV_64FC1) ;

	for( int i=0 ; i<vec_index.size() ; i++ )
	{
		A2.at<double>(i,0) = x[vec_index[i]] * x[vec_index[i]]  ;
		A2.at<double>(i,1) = x[vec_index[i]] ;
		A2.at<double>(i,2) = 1.0 ;
		
		B2.at<double>(i,0) = y[vec_index[i]] ;
	}

	cv::Mat A2_pinv(3,vec_index.size(),CV_64FC1) ;
	invert(A2, A2_pinv, cv::DECOMP_SVD);
	
	cv::Mat X = A2_pinv * B2 ;

	//cDrawing
	cv::Mat F = A*X ;
	printf("matrix F : cols =%d, rows=%d\n", F.cols, F.rows) ;

	int interval = 5 ;
	cv::Mat imgResult(100*interval,100*interval,CV_8UC3) ;
	imgResult = cv::Scalar(0) ;
	for( int iy=0 ; iy<100 ; iy++ )
	{
		cv::circle(imgResult, cv::Point(x[iy]*interval, y[iy]*interval) ,3, cv::Scalar(0,0,255), cv::FILLED) ;

		double data = F.at<double>(iy,0) ;

		cv::circle(imgResult, cv::Point(x[iy]*interval, data*interval) ,1, cv::Scalar(0,255,0), cv::FILLED) ;
	}
	cv::imshow("result", imgResult) ;
	cv::waitKey(0) ;
}

} // namespace geometry_transform
} // namespace opencv
} // namespace xslam