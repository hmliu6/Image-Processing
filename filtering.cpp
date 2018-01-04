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

cv::Mat image;

const char* imageDir = "boundaryCase";
// Here we use millimeter
const int fenceHeight = 2500;
const int kinectHeight = 1800;
const int fenceToKinect = 3000;
const int tolerance = 200;
const int maxDepthRange = 8000;
const int imageNumber = 1;
const int rodLength = 35;
const int rodWidth = 5;

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
    // Here we define image path as "sampleImage/test1-4500.png"
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

int getRodCoordinates(Mat image, int **whitePoints, int pointCount){
    // Initialize array with all zero to store satisfied x-coordinates
    int *rowWhiteNumber = (int *)malloc(image.rows * sizeof(int));
    int rowWhiteCount = 0, yCoordinates = 0;
    for(int i=0; i<10; i++)
        rowWhiteNumber[i] = -1;

    int currentX = 0, consecutiveCount = 0, lastHeight = 0;
    bool chosen = false, garbage = false;
    // Finding x-coordinate of inner circle in set of white points
    for(int i=0; i<pointCount; i++){
        // cout << "{" << whitePoints[1][i] << ", " << whitePoints[0][i] << "}" << endl;
        // All points are already sorted according to x-coordinate
        if(whitePoints[0][i] < image.rows/2){
            currentX = 0;
            continue;
        }
        if(currentX != whitePoints[1][i]){
            // Re-count if x-coordinate changed
            currentX = whitePoints[1][i];
            lastHeight = whitePoints[0][i];
            consecutiveCount = 0;
            chosen = false;
            garbage = false;
        }
        else{
            if(whitePoints[0][i] - lastHeight == 1 && !garbage){
                // Keep counting if we detected consecutive white
                consecutiveCount += 1;
                lastHeight = whitePoints[0][i];
            }
            else
                garbage = true;
            // Large consecutive count means that it is probably part of rod
            if(consecutiveCount >= rodLength && !chosen){
                // Mark it and denoted as chosen to prevent re-adding
                rowWhiteNumber[rowWhiteCount] = currentX;
                rowWhiteCount += 1;
                chosen = true;
            }
        }
    }

    if(rowWhiteCount > 0){
        // Take sum of average to become x-coordinate of centre
        // for(int i=0; i<rowWhiteCount; i++){
        //     cout << rowWhiteNumber[i] << endl;
        //     yCoordinates += rowWhiteNumber[i];
        // }
        // yCoordinates = int(yCoordinates / rowWhiteCount);

        // New Algorithm: Find width of all objects in line and ignore unreasonable width
        int temp = 1;
        for(int i=temp; i<rowWhiteCount; i++){
            if(rowWhiteNumber[i] - rowWhiteNumber[i-1] != 1){
                int width = rowWhiteNumber[i-1] - rowWhiteNumber[temp-1];
                if(width != 0 && width <= rodWidth){
                    // Simply take middle one in the consecutive numbers
                    yCoordinates = rowWhiteNumber[temp - 1 + int(width/2)];

                    // Free used objects to prevent overflow
                    free(rowWhiteNumber);
                    return yCoordinates;
                }
                else{
                    temp = i + 1;
                }
            }
        }
        // Free used objects to prevent overflow
        free(rowWhiteNumber);
        return -1;
    }
    else{
        free(rowWhiteNumber);
        return -1;
    }
}

int getCircleCoordinates(Mat image, int **whitePoints, int pointCount, int centreX){
    int listOfHeight = 0, countList = 0, centreY = 0;
    bool blackZone = false;
    for(int i=image.cols/2; i>=0; i--){
        int count = 0;
        for(int j=0; j<pointCount; j++){
            // Test every points if they are inside circle, Increase count
            int dY = pow((whitePoints[0][j] - i), 2);
            int dX = pow((whitePoints[1][j] - centreX), 2);
            // Circle that must have smaller radius than inner one
            if(dX + dY < pow(20, 2))
                count += 1;
        }
        // If we get enough amounts of count, it means that now it is looping in the black area
        if(countList > 10)
            blackZone = true;
        // Once we get a circle with too many white points, then we stop looping
        if(count >= 20 && blackZone)
            break;
        else if(count <= 10){
            listOfHeight += i;
            countList += 1;
        }
    }
    // Take sum of average to get y-coordinate of centre
    if(countList > 0){
        centreY = int(listOfHeight / countList);
        return centreY;
    }
    else
        return -1;
}

int getCircleRadius(int **whitePoints, int pointCount, Point centre){
    // Test for the maximum acceptable radius
    int largestRadius = 0;
    // 50 can be other values which is sufficiently large enough
    for(int i=0; i<50; i++){
        int count = 0;
        for(int j=0; j<pointCount; j++){
            // Test every points if they are inside circle, Increase count
            int dY = pow((whitePoints[0][j] - centre.y), 2);
            int dX = pow((whitePoints[1][j] - centre.x), 2);
            if(dX + dY < pow(i, 2))
                count += 1;
        }
        // Once we get a circle with too many white points, then we stop looping
        if(count > 20)
            break;
        // Keep storing largest radius value
        else if(count <= 10 && i > largestRadius){
            largestRadius = i;
        }
    }
    if(largestRadius > 0)
        return largestRadius;
    else
        return -1;
}

// Draw function for finding inner circle
void drawBoundingArea(Mat rawImage, Mat image, int **whitePoints, int pointCount){
    // Centre of bounding circle (x, y)
    Point centre;
    
    centre.x = getRodCoordinates(image, whitePoints, pointCount);
    if(centre.x == -1)
        return;

    centre.y = getCircleCoordinates(image, whitePoints, pointCount, centre.x);
    if(centre.y == -1)
        return;
    
    int largestRadius = getCircleRadius(whitePoints, pointCount, centre);
    if(largestRadius == -1)
        return;
    
    // Draw circle in raw image instead of processed image
    circle(rawImage, centre, largestRadius, CV_RGB(255, 255, 255), 2);
}

void preFiltering(cv::Mat rawImage, int lowerColorRange){
    // rawImage.copyTo(image);
    int *whitePoints[2], pointCount = 0;
    // whitePoints[0] = set of y-coordinates
    whitePoints[0] = (int *)malloc(10000 * sizeof(int));
    // whitePoints[1] = set of x-coordinates
    whitePoints[1] = (int *)malloc(10000 * sizeof(int));
    // Filter out unrelated pixels
    for(int j=0; j<image.cols; j++){
        for(int i=0; i<image.rows; i++){
            // Assume that the circle must be higher than image centre
            if(i >= image.cols/2)
                image.at<uchar>(i, j) = 0;
            // Trim out leftmost and rightmost 1/8 image to reduce noise
            else if(j <= image.rows/8)
                image.at<uchar>(i, j) = 0;
            else if(j >= image.rows* 7/8)
                image.at<uchar>(i, j) = 0;
            // Set all smaller than minimum colour value points to zero
            else if(image.at<uchar>(i, j) <= lowerColorRange)
                image.at<uchar>(i, j) = 0;
            else{
                // Set it to white and add to array for faster calculation
                image.at<uchar>(i, j) = 255;
                whitePoints[0][pointCount] = i;
                whitePoints[1][pointCount] = j;
                pointCount += 1;
            }
        }
    }
    // Keep processing if there is at least one point
    if(pointCount > 0)
        drawBoundingArea(rawImage, image, whitePoints, pointCount);
    free(whitePoints[0]);
    free(whitePoints[1]);
}

// Get a filtered image for finding bounding circle
void imageFopen(char *filePath, int lowerColorRange){
	cv::Mat rawImage;
	rawImage = imread(filePath, 1);

	if (!rawImage.data)
		cout << "Cannot find document!!!" << endl;
	else{
        // After opening files, convert to greyscale and pass to processing function
        // Here is only for reading image since image is stored in RGBA
        cvtColor(rawImage, image, COLOR_BGR2GRAY);
        preFiltering(rawImage, lowerColorRange);
		imshow("Test", rawImage);
		cvWaitKey(0);
	}
}

int main(int argc, char **argv){
    int lowerColor;
    colorCalculation(&lowerColor);
    char* filePath = pathParser(imageDir);
    imageFopen(filePath, lowerColor);
    return 0;
}