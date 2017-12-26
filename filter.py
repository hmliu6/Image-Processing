import cv2
import os
import math
import argparse


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
    parser.add_argument("-m", "--maxdepth", help="Maximum depth value using in kinect v2", type=int)
    parser.add_argument("-k", "--height", help="Height from ground to kinect", type=int, default=1800)
    parser.add_argument("-t", "--tolerance", help="Define allowance error range", type=int, default=300)
    parser.add_argument("-n", "--number", help="Define designated image file number", type=int, default=1)
    parser.add_argument("-d", "--distance", help="Define horizontal distance from fence to kinect\n"+
                                                 "Throwing Zone 2/3 to centre: 5930mm", type=int, default=4000)
    args = parser.parse_args()
    if (args.maxdepth):
        maxDepthRange = args.maxdepth
    else:
        parser.error('Please specify Maximum Depth Value with the -m or --maxdepth option')
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
    print upperDistance, lowerDistance

def imageProcess():
    # Read image from ImageTest folder
    imagePath = os.path.join(imageDir, ('test' + str(imageNumber) + '-' + str(maxDepthRange) + '.png'))
    rawImage = cv2.imread(imagePath, cv2.IMREAD_GRAYSCALE)
    # Prevent directly modifying original image
    image = rawImage

    for i in range (image.shape[0]): # traverses through height of the image
        for j in range (image.shape[1]): # traverses through width of the image
            # Since fence is always higher than kinect camera, we just take the upper depth map
            if (i >= image.shape[0]/2):
                image[i][j] = 0
            elif (image[i][j] <= lowerColor):
                image[i][j] = 0
            elif (image[i][j] >= upperColor):
                image[i][j] = 0

    cv2.imshow('image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


parsing()
colorCalculation()
imageProcess()