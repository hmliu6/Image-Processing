import cv2
import os
import math
import argparse
import numpy as np

imageDir = 'sampleImage'
# Here we use millimeter
fenceHeight = 2500
kinectHeight = 1800
fenceToKinect = 4000
tolerance = 300
maxDepthRange = 8000
imageNumber = 1
# Constant declaration above

def parsing():
    # Here is just parsing and give arguments to change defined values
    global maxDepthRange, kinectHeight, tolerance, imageNumber, fenceToKinect
    parser = argparse.ArgumentParser(prog = "python filter.py", description="Assume all dimesions are in millimeter", formatter_class = argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-m", "--maxdepth", help="Maximum depth value using in kinect v2", type=int, default=8000)
    parser.add_argument("-k", "--height", help="Height from ground to kinect", type=int, default=1800)
    parser.add_argument("-t", "--tolerance", help="Define allowance error range", type=int, default=300)
    parser.add_argument("-n", "--number", help="Define designated image file number", type=int, default=1)
    parser.add_argument("-d", "--distance", help="Define horizontal distance from fence to kinect\n"+
                                                 "Throwing Zone 2/3 to centre: 5930mm", type=int, default=4000)
    args = parser.parse_args()
    if (args.maxdepth):
        maxDepthRange = args.maxdepth
    if (args.height):
        kinectHeight = args.height
    if (args.tolerance):
        tolerance = args.tolerance
    if (args.number):
        imageNumber = args.number
    if (args.distance):
        fenceToKinect = args.distance

def colorCalculation():
    global upperColor, lowerColor
    # Pythagoras's theorem, a^2 + b^2 = c^2
    centreToKinect = math.sqrt(math.pow(fenceHeight - kinectHeight, 2) + math.pow(fenceToKinect, 2))
    # Get the bounding distance which fence lied within
    upperDistance = centreToKinect + tolerance
    lowerDistance = centreToKinect - tolerance
    # Since we get depth image by dividing 0-255 into number of depthRange pieces
    # So we use bounding distance to obtain corresponding colour range
    upperColor = int(255.0 * upperDistance/maxDepthRange)
    lowerColor = int(255.0 * lowerDistance/maxDepthRange)

def imageProcess():
    # Read image from ImageTest folder
    imagePath = os.path.join(imageDir, ('test' + str(imageNumber) + '-' + str(maxDepthRange) + '.png'))
    rawImage = cv2.imread(imagePath)
    # Prevent directly modifying original image
    image = cv2.cvtColor(rawImage, cv2.COLOR_BGR2GRAY)
    dummyImage = image.copy()

    # Min-X, Max-X, Min-Y, Max-Y
    boundingBox = [dummyImage.shape[1], 0, dummyImage.shape[0], 0]
    # Sum and count of white points
    pointSum = [0, 0]
    pointCount = 0
    # traverses through height of the image
    for i in range (dummyImage.shape[0]):
        # traverses through width of the image
        for j in range (dummyImage.shape[1]):
            # Since fence is always higher than kinect camera, we just take the upper depth map
            if (i >= dummyImage.shape[0]/2):
                dummyImage[i][j] = 0
            elif (dummyImage[i][j] <= lowerColor):
                dummyImage[i][j] = 0
            elif (dummyImage[i][j] >= upperColor):
                dummyImage[i][j] = 0
            else:
                dummyImage[i][j] = 255
                pointSum[0] += j
                pointSum[1] += i
                pointCount += 1
                if (boundingBox[0] > j):
                    boundingBox[0] = j
                if (boundingBox[1] < j):
                    boundingBox[1] = j
                if (boundingBox[2] > i):
                    boundingBox[2] = i
                if (boundingBox[3] < i):
                    boundingBox[3] = i

    pointSum[0] = int(pointSum[0]/pointCount)
    pointSum[1] = int(pointSum[1]/pointCount)
    smallestY = dummyImage.shape[0]
    # traverses through height of the image
    for i in range (pointSum[1], 0, -1):
        for j in range (pointSum[0]-10, pointSum[0]+10):
            if (dummyImage[i][j] == 255 and i < smallestY):
                smallestY = i
    boundingRadius = pointSum[1] - smallestY
    print boundingRadius
    print pointSum[0], pointSum[1]

    # Draw bounding circle
    cv2.circle(image,(pointSum[0], pointSum[1]), boundingRadius, (255, 255, 255), thickness=2, lineType=8, shift=0)

    cv2.imshow("output", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


parsing()
colorCalculation()
imageProcess()