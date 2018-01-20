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

cv::Mat rawImage;
cv::Mat imageForBall;

// Directories name which are stored images
const char* imageDir = "throwing-ball";

// Here we use millimeter
const int fenceHeight = 2500;
const int kinectHeight = 1800;
const int fenceToKinect = 3000;
const int tolerance = 200;
const int maxDepthRange = 8000;
const int rodLength = 35;
const int rodWidth = 7;

int imageNumber = 1;
const int medianBlurValue = 5;
const int cannyLower = 10;
const int cannyUpper = 150;
int detectedBall = 0, detectedIndex = 0;
vector<Point2f> ballCentre(20);

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
    // Here we define image path as "sampleImage/image1.png"
    pathLength = strlen(fileDir) + 1 + strlen("image") + int(log(imageNumber)) + strlen(".png") + 1;
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

void ballFilter(cv::Mat image){
    cv::Mat cannyEdge;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // Trim out lower half of image
    for(int j=0; j<image.cols; j++){
        for(int i=0; i<image.rows; i++){
            // Assume that the circle must be higher than image centre
            if(i >= image.rows/2)
                image.at<uchar>(i, j) = 0;
        }
    }

    // Function(sourceImage, destImage, params);
    medianBlur(imageForBall, imageForBall, 2 * medianBlurValue + 1);
    Canny(imageForBall, cannyEdge, cannyLower, cannyUpper);
    findContours(cannyEdge, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Draw all contours with filled colour
    Scalar color(255, 0, 0);
    cout << contours.size() << endl;
    // for(int i = 0; i < contours.size(); i++) // Iterate through each contour
    //     drawContours(rawImage, contours, i, color, CV_FILLED, 8, hierarchy);

    if(contours.size() > 0){
        // Get moments of contours
        vector<Moments> moment(contours.size());
        for(int i = 0; i < contours.size(); i++)
            moment[i] = moments(contours[i], false);

        // Get mass centre of contours
        vector<Point2f> massCentre(contours.size());
        for(int i = 0; i < contours.size(); i++)
            massCentre[i] = Point2f(moment[i].m10/moment[i].m00 , moment[i].m01/moment[i].m00);
        
        // Draw centre on image
        cout << "{ " << massCentre[0].x << ", " << massCentre[0].y << " }" << endl;
        cv::line(rawImage, cv::Point(massCentre[0].x - 5, massCentre[0].y), cv::Point(massCentre[0].x + 5, massCentre[0].y), Scalar(255, 255, 0), 2);
        cv::line(rawImage, cv::Point(massCentre[0].x, massCentre[0].y - 5), cv::Point(massCentre[0].x, massCentre[0].y + 5), Scalar(255, 255, 0), 2);

        // Record current first point to vector array
        ballCentre[detectedIndex] = massCentre[0];
        detectedIndex += 1;
        detectedBall = 1;
    }
    else{
        detectedBall -= 1;
    }

    // Reset vector array if cannot detect ball in consecutive 4 frames
    if(detectedBall == -3){
        detectedIndex = 0;
    }

    for(int i=1; i<detectedIndex; i++)
        cv::line(rawImage, ballCentre[i-1], ballCentre[i], Scalar(0, 255, 255), 2);
}

// Get a filtered image for finding bounding circle
void imageFopen(char *filePath){
	rawImage = imread(filePath, 1);

	if (!rawImage.data)
		cout << "Cannot find " << filePath << endl;
	else{
        // After opening files, convert to greyscale and pass to processing function
        // Here is only for reading image since image is stored in RGBA
        cvtColor(rawImage, imageForBall, COLOR_BGR2GRAY);
        ballFilter(imageForBall);
		imshow("Test", rawImage);
        imshow("Musked", imageForBall);
		int key = cvWaitKey(0);
        if(key == 13)
            exit(0);
	}
}

int main(int argc, char **argv){
    int lowerColor;
    imageNumber = 1;
    for(int i=0; i<70; i++){
        char* filePath = pathParser(imageDir);
        cout << filePath << endl;
        imageFopen(filePath);
        imageNumber += 1;
        cout << "---------------------" << endl;
    }
    return 0;
}