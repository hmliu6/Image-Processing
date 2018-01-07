#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <string>

#if defined(__WIN32__) || defined(__WIN64__)
   const char pathSlash = '\\';
#elif defined(__linux__) || defined(__APPLE__)
   const char pathSlash = '/';
#endif

using namespace std;
using namespace cv;

const char* imageDir = "throwing-ball";
// Here we use millimeter
const int fenceHeight = 2500;
const int kinectHeight = 1800;
const int fenceToKinect = 3000;
const int tolerance = 200;
const int maxDepthRange = 8000;
const int rodLength = 35;
const int rodWidth = 5;
int imageNumber = 26;

// Calculate minimum meaningful colour range
void colorCalculation(int *lowerColor){
    // Pythagoras's theorem, a^2 + b^2 = c^2
    double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
    double lowerDistance = centreToKinect - tolerance;
    // Map from depth value to grayscale
    *lowerColor = int(255.0 * lowerDistance/maxDepthRange);
}

// Only for image inputs in defined paths
char* pathParser(const char *fileDir){
    char *parsedPath;
    char buffer[10];
    int pathLength = 0;
    // Here we define image path as "throwing-ball/image1.png"
    pathLength = strlen(fileDir) + 1 + strlen("image") + int(log(imageNumber)) + 1 + strlen(".png");
    parsedPath = (char *)malloc(sizeof(char) * (pathLength+1));

    for(int i=0; i<strlen(fileDir); i++)
        parsedPath[i] = fileDir[i];
    
    parsedPath[strlen(fileDir)] = pathSlash;
    parsedPath[strlen(fileDir)+1] = '\0';
    
    std::strcat(parsedPath, "image");
    sprintf(buffer,"%d",imageNumber);
    std::strcat(parsedPath, buffer);
    std::strcat(parsedPath, ".png");

    parsedPath[pathLength] = '\0';
    return parsedPath;
}

// Get a filtered image for finding bounding circle
void imageFopen(char *shieldImagePath, char *baseImagePath, int lowerColorRange){
    cv::Mat shieldRawImage, shield2RawImage, baseRawImage, shieldImage, shield2Image, baseImage;
	shieldRawImage = imread(shieldImagePath, 1);
    // shield2RawImage = imread(shield2ImagePath, 1);
    baseRawImage = imread(baseImagePath, 1);

	if (!shieldRawImage.data || !baseRawImage.data)
		cout << "Cannot find document!!!" << endl;
	else{
        // After opening files, convert to greyscale and pass to processing function
        // Here is only for reading image since image is stored in RGBA
        cvtColor(shieldRawImage, shieldImage, COLOR_BGR2GRAY);
        // cvtColor(shield2RawImage, shield2Image, COLOR_BGR2GRAY);
        cvtColor(baseRawImage, baseImage, COLOR_BGR2GRAY);

        // Remove all pixels same in shield image and base image
        for(int j=0; j<shieldImage.cols; j++){
            for(int i=0; i<shieldImage.rows; i++){
                if(shieldImage.at<uchar>(i, j) > 0)
                    baseImage.at<uchar>(i, j) = 0;
                else if(baseImage.at<uchar>(i, j) > 0)
                    baseImage.at<uchar>(i, j) = 255;
            }
        }
        //

        // Canny Edge
        cv::Mat mEdge, mThres;
        medianBlur(baseImage, baseImage, 5); /*Just to smooth the Edges*/
        /*This is for Automatically calculating the Canny Threshold values*/
        double CannyAccThresh = threshold(baseImage, mThres, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
        double CannyThresh = 0.1 * CannyAccThresh;

        Canny(baseImage, mEdge, CannyThresh, CannyAccThresh); 

        Mat kernal_E, kernal_D;
        int erosion_size = 2,dilation_size = 2;
        kernal_E = getStructuringElement(MORPH_RECT, Size(2 * erosion_size + 1, 2 * erosion_size+1),
                                         Point(erosion_size, erosion_size));
        kernal_D = getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size+1),
                                         Point(dilation_size, dilation_size));
        dilate(mEdge, mEdge, kernal_D);//Just to make sure all the Edges are closed
        erode(mEdge, mEdge, kernal_E);

        // imshow("mCanny_Edge", mEdge);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        /// Find contours
        findContours(mEdge.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        /// Approximate contours to polygons + get bounding rects and circles
        vector<vector<Point> > contours_poly(contours.size());
        for(int i = 0; i < contours.size(); i++)
            approxPolyDP(Mat(contours[i]), contours_poly[i], 1, true);

        for(int i = 0; i < contours_poly.size(); i++)
            if(contours_poly[i].size() > 7)//Just a filter You can add more here
                drawContours(baseRawImage, contours_poly, i, Scalar(0, 255, 0), 2);

        imshow("mResult", baseRawImage);
        //

		// imshow("Test", baseImage);
		cvWaitKey(0);
	}
}

int main(int argc, char **argv){
    int lowerColor;
    colorCalculation(&lowerColor);
    char* shieldImage = pathParser(imageDir);
    for(int i=1; i<=44; i++){
        imageNumber = imageNumber + 1;
        char* baseImage = pathParser(imageDir);
        cout << baseImage << endl;
        imageFopen(shieldImage, baseImage, lowerColor);
    }
    return 0;
}