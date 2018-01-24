#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include <string>
#define IMAGE_FORMAT uchar

#if defined(__WIN32__) || defined(__WIN64__)
   const char pathSlash = '\\';
#elif defined(__linux__) || defined(__APPLE__)
   const char pathSlash = '/';
#endif

using namespace std;
using namespace cv;

cv::Mat rawImage, imageForBall;

typedef struct {
	cv::Point centre;
	int distance;
    int maxRadius;
} circleInfo;

typedef struct {
    cv::Point ballCentre;
    int zDistance;
} ballInfo;

typedef struct {
    cv::Point point1;
    int z1;
    cv::Point point2;
    int z2;
    cv::Point2d intersect;
    double twoPointDistance;
    double z_interpolation;
} intersectionPoint;

// Directories name which are stored images
const char* highestRodDir = "highestRod";
const char* sampleImageDir = "sampleImage";
const char* boundaryCaseDir = "boundaryCase";
const char* imageDir = "throwing-ball";

// Here we use millimeter
const int fenceHeight = 2500;
// Kinect height may be changed
const int kinectHeight = 1300;
// Smallest value here
const int fenceToKinect = 3000;
const int tolerance = 200;
const int maxDepthRange = 8000;
const int rodLength = 35;
const int rodWidth = 7;

int imageNumber = 1;
const int medianBlurValue = 5;
const int cannyLower = 10;
const int cannyUpper = 150;

// Global results of Goal and Ball tracing
int detectedBall = 0, recordedPos = 0, zPos = -1;
ballInfo *ballPath = new ballInfo[20];
circleInfo outputCircle;
bool circleExist = false;

pthread_t ballTracking;
pthread_mutex_t mutexLock = PTHREAD_MUTEX_INITIALIZER;

class Queue{
	public:
		Queue(int inputSize){
            // Constructor
			validElement = 0;
			circleArray = new circleInfo[inputSize];
			for(int i=0; i<inputSize; i++){
				circleArray[i].centre.x = -1;
				circleArray[i].centre.y = -1;
				circleArray[i].distance = -1;
			}
			arraySize = inputSize;
		}
        
		void enqueue(cv::Point input, int maxRadius){
            // Dequeue and enqueue input element if full
			if(validElement < arraySize){
				circleArray[validElement].centre.x = input.x;
				circleArray[validElement].centre.y = input.y;
				circleArray[validElement].distance = pow(input.x, 2) + pow(input.y, 2);
                circleArray[validElement].maxRadius = maxRadius;
				validElement += 1;
			}
			else{
				for(int i=1; i<arraySize; i++){
					circleArray[i - 1].centre.x = circleArray[i].centre.x;
					circleArray[i - 1].centre.y = circleArray[i].centre.y;
					circleArray[i - 1].distance = circleArray[i].distance;
                    circleArray[i - 1].maxRadius = circleArray[i].maxRadius;
				}
				circleArray[arraySize - 1].centre.x = input.x;
				circleArray[arraySize - 1].centre.y = input.y;
				circleArray[arraySize - 1].distance = pow(input.x, 2) + pow(input.y, 2);
                circleArray[arraySize - 1].maxRadius = maxRadius;
			}
		}

		circleInfo medianDistance(){
			int index;
			if(validElement == 1)
				return circleArray[validElement - 1];

            // Sort according to centre distance with (0, 0) and radius
			int *tempDistance = new int[validElement];
			for(int i=0; i<validElement; i++)
				tempDistance[i] = circleArray[i].distance * 1000 + circleArray[i].maxRadius;
			sort(tempDistance, tempDistance + validElement);

            // Take most reasonable one to be centre
            int desiredList[3], desiredValue;
            for(int i=0; i<3; i++)
                desiredList[i] = tempDistance[int(validElement / 2) - 1 + i];
            for(int i=0; i<3; i++)
                if(abs((desiredList[i] % 1000) - (desiredList[(i + 1) % 3] % 1000)) < 2){
                    if(desiredList[i] > desiredList[(i + 1) % 3])
                        desiredValue = desiredList[i];
                    else
                        desiredValue = desiredList[(i + 1) % 3];
                }

			for(index=0; index<validElement; index++)
				if(circleArray[index].distance * 1000 + circleArray[index].maxRadius == desiredValue)
					break;
			
            delete[] tempDistance;
			return circleArray[index];
		}

        void deleteAllocation(){
            delete[] circleArray;
        }

	private:
		circleInfo *circleArray;
		int validElement;
		int arraySize;
};

int arraySize = 10;
Queue *locationQueue = new Queue(arraySize);

// Calculate minimum meaningful colour range
void colorCalculation(int *lowerColor){
    // Pythagoras's theorem, a^2 + b^2 = c^2
    double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
    double lowerDistance = centreToKinect - tolerance;
    // Map from depth value to grayscale, change here if using raw 16 bits
    *lowerColor = int(255.0 * lowerDistance/maxDepthRange);
}

// Only for image inputs in defined paths
char* pathParser(const char *fileDir){
    char *parsedPath;
    char buffer[10];
    int pathLength = 0;
    // Here we define image path as "sampleImage/test1-4500.png"
    pathLength = strlen(fileDir) + 1 + strlen("image") + int(log(imageNumber)) + 6 + strlen(".png");
    parsedPath = new char[pathLength + 1];

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

int getRodCoordinates(int imageWidth, int **whitePoints, int pointCount){
    // Initialize array with all zero to store satisfied x-coordinates
    int *rowWhiteNumber = new int[imageWidth];
    int rowWhiteCount = 0, yCoordinates = 0;
    for(int i=0; i<10; i++)
        rowWhiteNumber[i] = -1;

    int currentX = 0, consecutiveCount = 0, lastHeight = 0;
    bool chosen = false, garbage = false;
    // Finding x-coordinate of inner circle in set of white points
    for(int i=0; i<pointCount; i++){
        // All points are already sorted according to x-coordinate
        if(whitePoints[0][i] < imageWidth / 2){
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
            // Give tolerance since poor accuracy for long distance
            if(whitePoints[0][i] - lastHeight <= 2 && !garbage){
                // Keep counting if we detected consecutive white
                consecutiveCount += 1;
                lastHeight = whitePoints[0][i];
            }
            // else
            //     garbage = true;
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
        // New Algorithm: Find width of all objects in line and ignore unreasonable width
        int temp = 1;
        for(int i=temp; i<rowWhiteCount; i++){
            if(rowWhiteNumber[i] - rowWhiteNumber[i-1] != 1 || (i == rowWhiteCount - 1 && rowWhiteNumber[i] - rowWhiteNumber[i-1] == 1)){
                // Numbers represent pixels, so width has to +1
                int width = rowWhiteNumber[i-1] - rowWhiteNumber[temp-1] + 1;
                if(i == rowWhiteCount - 1 && rowWhiteNumber[i] - rowWhiteNumber[i-1] == 1)
                    width = rowWhiteNumber[i] - rowWhiteNumber[temp-1] + 1;
                if(width > 1 && width <= rodWidth){
                    // Simply take middle one in the consecutive numbers
                    yCoordinates = rowWhiteNumber[temp - 1 + int(width/2)];

                    // Free used objects to prevent overflow
                    delete[] rowWhiteNumber;
                    return yCoordinates;
                }
                else{
                    temp = i + 1;
                }
            }
        }
        // Free used objects to prevent overflow
        delete[] rowWhiteNumber;
        return -1;
    }
    else{
        delete[] rowWhiteNumber;
        return -1;
    }
}

int getCircleCoordinates(int imageHeight, int **whitePoints, int pointCount, int centreX){
    int listOfHeight = 0, countList = 0, centreY = 0;
    bool blackZone = false;
    for(int i=imageHeight; i>=0; i--){
        int count = 0;
        for(int j=0; j<pointCount; j++){
            // Test every points if they are inside circle, Increase count
            int dY = pow((whitePoints[0][j] - i), 2);
            int dX = pow((whitePoints[1][j] - centreX), 2);
            // Circle that must have smaller radius than inner one
            if(dX + dY < pow(15, 2))
                count += 1;
        }
        // If we get enough amounts of count, it means that now it is looping in the black area
        if(countList > 10)
            blackZone = true;
        // Once we get a circle with too many white points, then we stop looping
        if(count > 10 && blackZone)
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

int getCircleRadius(int **whitePoints, int pointCount, cv::Point centre){
    // Test for the maximum acceptable radius
    int largestRadius = 0, threshold = 20;
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
        if(count > threshold)
            break;
        // Keep storing largest radius value
        else if(count <= threshold && i > largestRadius){
            largestRadius = i;
        }
    }
    if(largestRadius > 0)
        return largestRadius;
    else
        return -1;
}

// Draw function for finding inner circle
void drawBoundingArea(cv::Mat rawImage, cv::Mat rodImage, int **whitePoints, int pointCount){
    // Centre of bounding circle (x, y)
    cv::Point centre;

    if (pointCount == 0) {
		cout << "Not found" << endl;
		return;
	}
    
    centre.x = getRodCoordinates(rodImage.rows, whitePoints, pointCount);
    if(centre.x == -1){
        cout << "Cannot locate rod" << endl;
        return;
    }

    centre.y = getCircleCoordinates(rodImage.cols / 2, whitePoints, pointCount, centre.x);
    if(centre.y == -1){
        cout << "Cannot find centre" << endl;
        return;
    }

    int largestRadius = getCircleRadius(whitePoints, pointCount, centre);
	if (largestRadius == -1) {
		cout << "Cannot find radius" << endl;
		return;
	}
    
    locationQueue->enqueue(centre, largestRadius);
	outputCircle = locationQueue->medianDistance();

	// Draw circle in raw image instead of processed image
    if(outputCircle.centre.x > 0 && outputCircle.centre.y > 0 && outputCircle.maxRadius > 0){
        circleExist = true;
        pthread_mutex_lock(&mutexLock);
        circle(rawImage, outputCircle.centre, outputCircle.maxRadius, CV_RGB(255, 255, 255), 2);
        pthread_mutex_unlock(&mutexLock);

        // Locate z-Position for goal circle
        for(int j=0; j<pointCount; j++){
            // Test every points if they are on circle
            int dY = pow((whitePoints[0][j] - outputCircle.centre.y), 2);
            int dX = pow((whitePoints[1][j] - outputCircle.centre.x), 2);
            if(dX + dY == pow(outputCircle.maxRadius, 2)){
                int tempPosition = int(rodImage.at<IMAGE_FORMAT>(whitePoints[0][j], whitePoints[1][j]));
                if(tempPosition > zPos)
                    zPos = tempPosition;
            }
        }
    }
}

void preFiltering(cv::Mat rawImage, cv::Mat rodImage, int lowerColorRange){
    int *whitePoints[2], pointCount = 0;
    // Reset to -1 since it will be updated each frame
    outputCircle.centre = cv::Point(-1, -1);
    outputCircle.maxRadius = -1;
    // whitePoints[0] = set of y-coordinates
    whitePoints[0] = new int[100000];
    // whitePoints[1] = set of x-coordinates
    whitePoints[1] = new int[100000];
    // Filter out unrelated pixels, change here if using raw 16 bits
    for(int j=0; j<rodImage.cols; j++){
        for(int i=0; i<rodImage.rows; i++){
            // Assume that the circle must be higher than image centre
            if(i >= rodImage.cols/2)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            // Trim out leftmost and rightmost 1/8 image to reduce noise
            else if(j <= rodImage.rows/8)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            else if(j >= rodImage.rows* 7/8)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            // Set all smaller than minimum colour value points to zero
            else if(rodImage.at<IMAGE_FORMAT>(i, j) <= lowerColorRange)
                rodImage.at<IMAGE_FORMAT>(i, j) = 0;
            else{
                // Set it to white and add to array for faster calculation
                if(pointCount < 100000){
                whitePoints[0][pointCount] = i;
                whitePoints[1][pointCount] = j;
                pointCount += 1;
            }
        }
    }
    }
    // Keep processing if there is at least one point
    if(pointCount > 0)
        drawBoundingArea(rawImage, rodImage, whitePoints, pointCount);
    delete[] whitePoints[0];
    delete[] whitePoints[1];
}

void goalDetection(){
    bool result = false, intersection = false;
    double threshold = 0.1;
    int pointCount = 0;
    intersectionPoint *pointsOnLine = new intersectionPoint[3];
    for(int i=1; i<recordedPos; i++){
        // The ball should fly with increasing z-distance
        // cout << ballPath[i].zDistance << endl;
        if(ballPath[i].zDistance - ballPath[i - 1].zDistance < 0)
            result = false;

        // At least one point intersection between circle and path only considering xy-plane
        // Here Point(x, y) is correct
        double lineSlope, lineIntercept, aCoefficient, bCoefficient, cCoefficient, delta;
        lineSlope = (ballPath[i].ballCentre.y - ballPath[i - 1].ballCentre.y) / (ballPath[i].ballCentre.x - ballPath[i - 1].ballCentre.x);
        lineIntercept = -1 * lineSlope * ballPath[i - 1].ballCentre.x + ballPath[i - 1].ballCentre.y;
        aCoefficient = 1 + pow(lineSlope, 2);
        bCoefficient = 2.0 * lineSlope * (lineIntercept - outputCircle.centre.y) - 2.0 * outputCircle.centre.x;
        cCoefficient = pow(outputCircle.centre.x, 2) + pow(lineIntercept - outputCircle.centre.y, 2) - pow(outputCircle.maxRadius, 2);

        delta = pow(bCoefficient, 2) - 4.0 * aCoefficient * cCoefficient;
        if(delta >= 0){
            cv::Point2d intersect1, intersect2;
            // Retrieve two intersection points
            intersect1.x = (-1 * bCoefficient + sqrt(delta)) / (2.0 * aCoefficient);
            intersect1.y = lineSlope * intersect1.x + lineIntercept;
            intersect2.x = (-1 * bCoefficient - sqrt(delta)) / (2.0 * aCoefficient);
            intersect2.y = lineSlope * intersect2.x + lineIntercept;

            double twoPointDistance, intersect1DistanceSum, intersect2DistanceSum;
            // Check the solutions are within line segments or not
            twoPointDistance = sqrt(pow(ballPath[i].ballCentre.x - ballPath[i - 1].ballCentre.x, 2) + pow(ballPath[i].ballCentre.y - ballPath[i - 1].ballCentre.y, 2));
            // If distance of new point with two points is larger than distance of two points
            intersect1DistanceSum = sqrt(pow(intersect1.x - ballPath[i].ballCentre.x, 2) + pow(intersect1.y - ballPath[i].ballCentre.y, 2))
                                  + sqrt(pow(intersect1.x - ballPath[i - 1].ballCentre.x, 2) + pow(intersect1.y - ballPath[i - 1].ballCentre.y, 2));
            intersect2DistanceSum = sqrt(pow(intersect2.x - ballPath[i].ballCentre.x, 2) + pow(intersect2.y - ballPath[i].ballCentre.y, 2))
                                  + sqrt(pow(intersect2.x - ballPath[i - 1].ballCentre.x, 2) + pow(intersect2.y - ballPath[i - 1].ballCentre.y, 2));
            
            // Return only points lied within line segments
            if(fabs(intersect1DistanceSum - twoPointDistance) < threshold){
                // cv::line(rawImage, cv::Point(intersect1.x - 5, intersect1.y), cv::Point(intersect1.x + 5, intersect1.y), Scalar(255, 255, 0), 2);
                // cv::line(rawImage, cv::Point(intersect1.x, intersect1.y - 5), cv::Point(intersect1.x, intersect1.y + 5), Scalar(255, 255, 0), 2);
                pointsOnLine[pointCount].point1 = ballPath[i - 1].ballCentre;
                pointsOnLine[pointCount].z1 = ballPath[i - 1].zDistance;
                pointsOnLine[pointCount].point2 = ballPath[i].ballCentre;
                pointsOnLine[pointCount].z2 = ballPath[i].zDistance;
                pointsOnLine[pointCount].intersect = intersect1;
                pointsOnLine[pointCount].twoPointDistance = twoPointDistance;
                pointCount += 1;
            }
            else if(fabs(intersect2DistanceSum - twoPointDistance) < threshold){
                // cv::line(rawImage, cv::Point(intersect2.x - 5, intersect2.y), cv::Point(intersect2.x + 5, intersect2.y), Scalar(255, 255, 0), 2);
                // cv::line(rawImage, cv::Point(intersect2.x, intersect2.y - 5), cv::Point(intersect2.x, intersect2.y + 5), Scalar(255, 255, 0), 2);
                pointsOnLine[pointCount].point1 = ballPath[i - 1].ballCentre;
                pointsOnLine[pointCount].z1 = ballPath[i - 1].zDistance;
                pointsOnLine[pointCount].point2 = ballPath[i].ballCentre;
                pointsOnLine[pointCount].z2 = ballPath[i].zDistance;
                pointsOnLine[pointCount].intersect = intersect2;
                pointsOnLine[pointCount].twoPointDistance = twoPointDistance;
                pointCount += 1;
            }
        }

    }

    if(pointCount < 2)
        result = false;
    else{
        // Interpolate all points lied on line
        for(int i=0; i<pointCount; i++){
            int zDiff = pointsOnLine[i].z1 - pointsOnLine[i].z2;
            double distanceWithPoint1 = sqrt(pow(pointsOnLine[i].point1.x - pointsOnLine[i].intersect.x, 2) + pow(pointsOnLine[i].point1.y - pointsOnLine[i].intersect.y, 2));
            pointsOnLine[i].z_interpolation = pointsOnLine[i].z1 - zDiff * distanceWithPoint1 / pointsOnLine[i].twoPointDistance;
            cout << pointsOnLine[i].z_interpolation << endl;
        }
        // z-value: pointsOnLine[0].z_interpolation < zPos < pointsOnLine[1].z_interpolation
        cout << "Rod Position: " << zPos << endl;
        if(pointsOnLine[0].z_interpolation < zPos && zPos < pointsOnLine[1].z_interpolation)
            result = true;
    }

    // Put text on displayed image
    if(result == true && recordedPos > 0)
        putText(rawImage, string("Goal"), Point(430, 30), 0, 1, Scalar(0, 127, 255), 2);
    else if(result == false && recordedPos > 0)
        putText(rawImage, string("Fail"), Point(430, 30), 0, 1, Scalar(0, 127, 255), 2);
    delete[] pointsOnLine;
}

void *ballFilter(void *input){
    cv::Mat cannyEdge, temp;
    vector<vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;

    // Trim out lower half of image
    for(int j=0; j<imageForBall.cols; j++){
        for(int i=0; i<imageForBall.rows; i++){
            // Assume that the ball must be higher than image centre, change here if using raw 16 bits
            if(i >= imageForBall.rows/2)
                imageForBall.at<IMAGE_FORMAT>(i, j) = 0;
        }
    }

    // Function(sourceImage, destImage, params);
    medianBlur(imageForBall, temp, 2 * medianBlurValue + 1);
    Canny(temp, cannyEdge, cannyLower, cannyUpper);
    findContours(cannyEdge, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Draw all contours with filled colour
    Scalar color(255, 0, 0);
    // for(int i = 0; i < contours.size(); i++) // Iterate through each contour
    //     drawContours(rawImage, contours, i, color, CV_FILLED, 8, hierarchy);

    if(contours.size() > 0){
        // Get moments of contours
        vector<Moments> moment(contours.size());
        for(int i = 0; i < contours.size(); i++)
            moment[i] = moments(contours[i], false);

        // Get mass centre of contours
        vector<Point2f> massCentre(contours.size());
        int massCentreCounter = 0;
        for(int i = 0; i < contours.size(); i++){
            if(moment[i].m00 > 0){
                massCentre[massCentreCounter] = Point2f(moment[i].m10/moment[i].m00, moment[i].m01/moment[i].m00);
                massCentreCounter += 1;
            }
        }
        // Draw centre on image
        // cout << "{ " << massCentre[0].x << ", " << massCentre[0].y << " }" << endl;
        // cv::line(rawImage, cv::Point(massCentre[0].x - 5, massCentre[0].y), cv::Point(massCentre[0].x + 5, massCentre[0].y), Scalar(255, 255, 0), 2);
        // cv::line(rawImage, cv::Point(massCentre[0].x, massCentre[0].y - 5), cv::Point(massCentre[0].x, massCentre[0].y + 5), Scalar(255, 255, 0), 2);

        // Record current first point to vector array
        ballPath[recordedPos].ballCentre = cv::Point(massCentre[0]);
        ballPath[recordedPos].zDistance = int(imageForBall.at<IMAGE_FORMAT>(int(massCentre[0].y), int(massCentre[0].x)));
        cout << "zDistance : " << (int)imageForBall.at<IMAGE_FORMAT>(int(massCentre[0].y), int(massCentre[0].x)) << endl;

        recordedPos += 1;
        detectedBall = 1;
    }
    else{
        detectedBall -= 1;
    }

    // Reset vector array if cannot detect ball in consecutive 5 frames
    if(detectedBall == -4){
        recordedPos = 0;
    }

    // Draw trace line with mutex lock to achieve mutually exclusive
    pthread_mutex_lock(&mutexLock);
    for(int i=1; i<recordedPos; i++)
        cv::line(rawImage, ballPath[i-1].ballCentre, ballPath[i].ballCentre, Scalar(0, 255, 255), 2);
    pthread_mutex_unlock(&mutexLock);

    pthread_exit(NULL);
}

void imageProcessing(cv::Mat rodImage, int lowerColorRange){
    rodImage.copyTo(imageForBall);

    // Create thread to perform two separated tasks
    circleExist = false;
    pthread_create(&ballTracking, NULL, ballFilter, NULL);
    preFiltering(rawImage, rodImage, lowerColorRange);
    pthread_join(ballTracking, NULL);

    if(detectedBall == -3 && recordedPos > 0)
        goalDetection();

    zPos = -1;
}

// Get a filtered image for finding bounding circle
void imageFopen(char *filePath, int lowerColorRange){
    cv::Mat rodImage;
	rawImage = imread(filePath, 1);

    pthread_mutex_init(&mutexLock, NULL);

	if (!rawImage.data)
		cout << "Cannot find " << filePath << endl;
	else{
        // After opening files, convert to greyscale and pass to processing function
        // Here is only for reading image since image is stored in RGBA
        cvtColor(rawImage, rodImage, COLOR_BGR2GRAY);
        imageProcessing(rodImage, lowerColorRange);
		imshow("Test", rawImage);
		cvWaitKey(0);
	}
    rawImage.release();
    imageForBall.release();
    rodImage.release();
}

int main(int argc, char **argv){
    int lowerColor;
    colorCalculation(&lowerColor);
    imageNumber = 1;
    for(int i=0; i<77; i++){
        char* filePath = pathParser(imageDir);
        cout << filePath << endl;
        imageFopen(filePath, lowerColor);
        imageNumber += 1;
        cout << "---------------------" << endl;
        delete[] filePath;
    }
    locationQueue->deleteAllocation();
    delete locationQueue;
    return 0;
}