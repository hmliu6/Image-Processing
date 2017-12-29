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

const char* imageDir = "sampleImage";
// Here we use millimeter
const int fenceHeight = 2500;
const int kinectHeight = 1800;
const int fenceToKinect = 3000;
const int tolerance = 200;
const int maxDepthRange = 8000;
const int imageNumber = 1;

void colorCalculation(int *lowerColor){
    // Pythagoras's theorem, a^2 + b^2 = c^2
    double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
    double lowerDistance = centreToKinect - tolerance;
    *lowerColor = int(255.0 * lowerDistance/maxDepthRange);
}

char* pathParser(const char *fileDir){
    char *parsedPath;
    char buffer[10];
    int pathLength = 0;
    pathLength = strlen(fileDir) + 1 + strlen("test") + 6 + strlen(".png");
    parsedPath = (char *)malloc(sizeof(char) * (pathLength+1));

    for(int i=0; i<strlen(fileDir); i++)
        parsedPath[i] = fileDir[i];
    
    parsedPath[strlen(fileDir)] = pathSlash;
    parsedPath[strlen(fileDir)+1] = '\0';
    
    std::strcat(parsedPath, "test");
    sprintf(buffer,"%d",imageNumber);
    std::strcat(parsedPath, buffer);
    std::strcat(parsedPath, "-");
    sprintf(buffer,"%d",maxDepthRange);
    std::strcat(parsedPath, buffer);
    std::strcat(parsedPath, ".png");

    parsedPath[pathLength] = '\0';
    return parsedPath;
}

void drawBoundingArea(Mat rawImage, Mat image, int **whitePoints, int pointCount){
    // Centre of bounding circle (x, y)
    Point centre;
    centre.x = 0; centre.y = 0;
    int *rowWhiteNumber = (int *)malloc(image.rows * sizeof(int));
    int rowWhiteCount = 0;
    for(int i=0; i<10; i++)
        rowWhiteNumber[i] = -1;

    int currentX = 0, consecutiveCount = 0, lastHeight = 0;
    bool chosen = false, garbage = false;
    for(int i=0; i<pointCount; i++){
        // cout << "{" << whitePoints[1][i] << ", " << whitePoints[0][i] << "}" << endl;
        if(whitePoints[0][i] < image.rows/2){
            currentX = 0;
            continue;
        }
        if(currentX != whitePoints[1][i]){
            currentX = whitePoints[1][i];
            lastHeight = whitePoints[0][i];
            chosen = false;
            garbage = false;
        }
        else{
            if(whitePoints[0][i] - lastHeight == 1 && !garbage){
                consecutiveCount += 1;
                lastHeight = whitePoints[0][i];
            }
            else
                garbage = true;
            if(consecutiveCount >= 20 && !chosen){
                rowWhiteNumber[rowWhiteCount] = currentX;
                rowWhiteCount += 1;
                chosen = true;
            }
        }
    }

    if(rowWhiteCount > 0){
        for(int i=0; i<rowWhiteCount; i++){
            centre.x += rowWhiteNumber[i];
            // cout << rowWhiteNumber[i] << endl;
        }
        centre.x = int(centre.x / rowWhiteCount);
    }
    cout << centre.x << endl;

    int listOfHeight = 0, countList = 0;
    bool blackZone = false;
    for(int i=image.cols/2; i>=0; i--){
        int count = 0;
        for(int j=0; j<pointCount; j++){
            int dY = pow((whitePoints[0][j] - i), 2);
            int dX = pow((whitePoints[1][j] - centre.x), 2);
            if(dX + dY < pow(20, 2))
                count += 1;
        }
        if(countList > 10)
            blackZone = true;
        if(count >= 20 && blackZone)
            break;
        else if(count <= 10){
            listOfHeight += i;
            countList += 1;
        }
    }
    if(countList > 0)
        centre.y = int(listOfHeight / countList);
    int largestRadius = 0;
    for(int i=0; i<50; i++){
        int count = 0;
        for(int j=0; j<pointCount; j++){
            int dY = pow((whitePoints[0][j] - centre.y), 2);
            int dX = pow((whitePoints[1][j] - centre.x), 2);
            if(dX + dY < pow(i, 2))
                count += 1;
        }
        if(count > 20)
            break;
        else if(count <= 10 && i > largestRadius){
            largestRadius = i;
        }
    }
    circle(rawImage, centre, largestRadius, CV_RGB(255, 255, 255), 2);
}

void preFiltering(char *filePath, int lowerColorRange){
	cv::Mat rawImage, image;
	rawImage = imread(filePath, 1);

	if (!rawImage.data)
		cout << "Cannot find document!!!" << endl;
	else{
        int *whitePoints[2], pointCount = 0;
        whitePoints[0] = (int *)malloc(10000 * sizeof(int));
        whitePoints[1] = (int *)malloc(10000 * sizeof(int));
        // After opening files, convert to greyscale and pass to processing function
		cvtColor(rawImage, image, COLOR_BGR2GRAY);
        // Filter out unrelated pixels
        for(int j=0; j<image.cols; j++){
            for(int i=0; i<image.rows; i++){
                if(i >= image.cols/2)
                    image.at<uchar>(i, j) = 0;
                else if(j <= image.rows/5)
                    image.at<uchar>(i, j) = 0;
                else if(j >= image.rows* 4/5)
                    image.at<uchar>(i, j) = 0;
                else if(image.at<uchar>(i, j) <= lowerColorRange)
                    image.at<uchar>(i, j) = 0;
                else{
                    image.at<uchar>(i, j) = 255;
                    whitePoints[0][pointCount] = i;
                    whitePoints[1][pointCount] = j;
                    pointCount += 1;
                }
            }
        }
        if(pointCount > 0)
            drawBoundingArea(rawImage, image, whitePoints, pointCount);
        
		imshow("Test", rawImage);
		cvWaitKey(0);
	}
}

int main(int argc, char **argv){
    int upperColor, lowerColor;
    // Pass by reference
    colorCalculation(&lowerColor);

    char* filePath = pathParser(imageDir);
    preFiltering(filePath, lowerColor);

    return 0;
}