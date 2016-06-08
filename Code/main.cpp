//
//  main.cpp
//  Lab4
//
//  Created by drinking on 12/30/15.
//  Copyright © 2015 drinking. All rights reserved.
//

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;
int n_boards = 0;
const int board_dt = 20;
int board_w;
int board_h;

char argv[4][100];
string str[4];

int main(int argc, const char * argv[]) {
	/*
    cin >> str[1] >> str[2] >> str[3];
    argv[1] = str[1].c_str();
    argv[2] = str[2].c_str();
    argv[3] = str[3].c_str();
	*/
    board_w = atoi(argv[1]);
    board_h = atoi(argv[2]);
    n_boards = atoi(argv[3]);
    int board_n = board_w*board_h;
    CvSize board_sz = cvSize(board_w,board_h);
    CvCapture* capture = cvCreateCameraCapture(0);
    assert(capture);
    
    cvNamedWindow("Calibration");
    CvMat* image_points = cvCreateMat(n_boards*board_n, 2, CV_32FC1);
    CvMat* object_points = cvCreateMat(n_boards*board_n, 3, CV_32FC1);
    CvMat* point_counts = cvCreateMat(n_boards, 1, CV_32SC1);
    CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
    CvMat* distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);
    
    CvPoint2D32f* corners = new CvPoint2D32f[board_n];
    int corner_count;
    int successes = 0;
    int step,frame = 0;
    
    IplImage *image = cvQueryFrame(capture);
    IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);
    while(successes < n_boards)
    {
        if (frame++ % board_dt == 0) {
            int found = cvFindChessboardCorners(image, board_sz, corners,&corner_count,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            cvCvtColor(image, gray_image, CV_BGR2GRAY);
            cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
            
            cvDrawChessboardCorners(image, board_sz, corners, corner_count, found);
            cvShowImage("Calibration", image);
            if( corner_count == board_n )
            {
                step = successes*board_n;
                for(int i=step,j=0;j<board_n;i++,j++)
                {
                    CV_MAT_ELEM(*image_points, float, i, 0) = corners[j].x;
                    CV_MAT_ELEM(*image_points, float, i, 1) = corners[j].y;
                    CV_MAT_ELEM(*object_points, float, i, 0) = j/board_w;
                    CV_MAT_ELEM(*object_points, float, i, 1) = j%board_w;
                    CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
                }
                CV_MAT_ELEM(*point_counts, int, successes,0) = board_n;
                successes++;
            }
        }
        
        //pause & unpause
       // cout << "wait.." << endl;
        int c = cvWaitKey(15);
        if( c== 'p' )
        {
            c = 0;
            while( c!='p' && c!=27 ) c = cvWaitKey(250);
        }
      //  cout << "wait finished.."  << endl;
        if( c == 27 ) return 0;
        image = cvQueryFrame(capture);
    }
    
    CvMat* object_points2 = cvCreateMat(successes*board_n, 3, CV_32FC1);
    CvMat* image_points2 = cvCreateMat(successes*board_n, 2, CV_32FC1);
    CvMat* point_counts2 = cvCreateMat(successes,1, CV_32SC1);
    for(int  i=0;i<successes*board_n;i++)
    {
        CV_MAT_ELEM(*image_points2, float, i, 0) = CV_MAT_ELEM(*image_points, float, i, 0);
        CV_MAT_ELEM(*image_points2, float, i, 1) = CV_MAT_ELEM(*image_points, float, i, 1);
        CV_MAT_ELEM(*object_points2, float, i, 0) = CV_MAT_ELEM(*object_points, float, i, 0);
        CV_MAT_ELEM(*object_points2, float, i, 1) = CV_MAT_ELEM(*object_points, float, i, 1);
        CV_MAT_ELEM(*object_points2, float, i, 2) = CV_MAT_ELEM(*object_points, float, i, 2);
    }
    for(int i=0;i<successes;i++)
        CV_MAT_ELEM(*point_counts2, int, i, 0) = CV_MAT_ELEM(*point_counts, int, i, 0);
    cvReleaseMat(&object_points);
    cvReleaseMat(&image_points);
    cvReleaseMat(&point_counts);
    
    CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1.0f;
    CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1.0f;
    
    cvCalibrateCamera2(object_points2, image_points2, point_counts2, cvGetSize(image), intrinsic_matrix, distortion_coeffs);
    
    cvSave("Intrinsics.xml", intrinsic_matrix);
    cvSave("Distortion.xml", distortion_coeffs);
    
    CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
    CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");
    
    IplImage *mapx = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage *mapy = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    
    CvPoint2D32f objPoints[4],imgPoints[4];
    cvInitUndistortMap(intrinsic, distortion, mapx, mapy);
    
    CvMat* H;
    float Z = 25;
    cvNamedWindow("birdview");
    /*
    while(image)
    {
        IplImage* t = cvCloneImage(image);
        cvRemap(t, image, mapx,mapy);
        int found = cvFindChessboardCorners(image, board_sz, corners,&corner_count,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if(!found)
        {
            image = cvQueryFrame(capture);
            continue;
        }
        cvCvtColor(image, gray_image, CV_BGR2GRAY);
        cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
        
        objPoints[0].x = 0; objPoints[0].y = 0;
        objPoints[1].x = board_w-1; objPoints[1].y = 0;
        objPoints[2].x = 0; objPoints[2].y = board_h-1;
        objPoints[3].x = board_w-1; objPoints[3].y = board_h-1;
        
        imgPoints[0] = corners[0];
        imgPoints[1] = corners[board_w-1];
        imgPoints[2] = corners[(board_h-1)*board_w];
        imgPoints[3] = corners[board_h*board_w-1];
        
        cvCircle(image, cvPointFrom32f(imgPoints[0]), 9, CV_RGB(255, 0, 0),3);
        cvCircle(image, cvPointFrom32f(imgPoints[1]), 9, CV_RGB(0, 255, 0),3);
        cvCircle(image, cvPointFrom32f(imgPoints[2]), 9, CV_RGB(0, 0, 255),3);
        cvCircle(image, cvPointFrom32f(imgPoints[3]), 9, CV_RGB(255, 255, 0),3);
        
        H = cvCreateMat(3, 3, CV_32F);
        cvGetPerspectiveTransform(objPoints, imgPoints, H);
        CV_MAT_ELEM(*H, float, 2, 2) = Z;
        

        
        IplImage *bird_image = cvCloneImage(image);
      //  cvShowImage("Calibration", image);
        cvWarpPerspective(image, bird_image, H,CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
        cvShowImage("birdview", bird_image);
        cvReleaseImage(&bird_image);
        int c = cvWaitKey(15);
        if( c == 'u' ) Z+=0.5;
        if( c == 'd' ) Z-=0.5;
        if( c == 27 ) break;
        image = cvQueryFrame(capture);
    }
    */
    //IplImage* img = cvLoadImage("/Users/drinkingcoder/Documents/university/Vision/Labs/Lab4/Lab4/sample.JPG");
    IplImage* img = cvLoadImage(argv[4]);
    cvResize(img, image);
    IplImage* t = cvCloneImage(image);
    cvRemap(t, image, mapx,mapy);
    int found = cvFindChessboardCorners(image, board_sz, corners,&corner_count,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if(!found)
    {
        cout << "Can't find!" << endl;
        return 0;
    }
    cvCvtColor(image, gray_image, CV_BGR2GRAY);
    cvFindCornerSubPix(gray_image, corners, corner_count, cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
    
    objPoints[0].x = 0; objPoints[0].y = 0;
    objPoints[1].x = board_w-1; objPoints[1].y = 0;
    objPoints[2].x = 0; objPoints[2].y = board_h-1;
    objPoints[3].x = board_w-1; objPoints[3].y = board_h-1;
    
    imgPoints[0] = corners[0];
    imgPoints[1] = corners[board_w-1];
    imgPoints[2] = corners[(board_h-1)*board_w];
    imgPoints[3] = corners[board_h*board_w-1];
    
    cvCircle(image, cvPointFrom32f(imgPoints[0]), 9, CV_RGB(255, 0, 0),3);
    cvCircle(image, cvPointFrom32f(imgPoints[1]), 9, CV_RGB(0, 255, 0),3);
    cvCircle(image, cvPointFrom32f(imgPoints[2]), 9, CV_RGB(0, 0, 255),3);
    cvCircle(image, cvPointFrom32f(imgPoints[3]), 9, CV_RGB(255, 255, 0),3);
    
    H = cvCreateMat(3, 3, CV_32F);
    cvGetPerspectiveTransform(objPoints, imgPoints, H);
    int key = 0;
    IplImage *bird_image = cvCloneImage(image);

    while(key!=27)
    {
        CV_MAT_ELEM(*H, float, 2, 2) = Z;
        cvShowImage("Calibration", image);
        cvWarpPerspective(image, bird_image, H,CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
        cvShowImage("birdview", bird_image);
        key= cvWaitKey();
        if( key == 'u' ) Z+=0.5;
        if( key == 'd' ) Z-=0.5;
        if( key == 27 ) break;
    }

    return 0;
}
