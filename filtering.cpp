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
const int fenceToKinect = 4000;
const int tolerance = 300;
const int maxDepthRange = 8000;
const int imageNumber = 1;

void colorCalculation(int *upperColor, int *lowerColor){
    // Pythagoras's theorem, a^2 + b^2 = c^2
    double centreToKinect = sqrt(pow(fenceHeight - kinectHeight, 2) + pow(fenceToKinect, 2));
    // Get the bounding distance which fence lied within
    double upperDistance = centreToKinect + tolerance;
    double lowerDistance = centreToKinect - tolerance;
    // Since we get depth image by dividing 0-255 into number of depthRange pieces
    // So we use bounding distance to obtain corresponding colour range
    *upperColor = int(255.0 * upperDistance/maxDepthRange);
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

int main(int argc, char **argv){
    int upperColor, lowerColor;
    // Pass by reference
    colorCalculation(&upperColor, &lowerColor);

    char* filePath = pathParser(imageDir);
    cout << filePath << endl;

    return 0;
}