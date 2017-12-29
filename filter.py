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
    upperColor = 255
    lowerColor = int(255.0 * lowerDistance/maxDepthRange)

def imageProcess():
    # Read image from ImageTest folder
    imagePath = os.path.join(imageDir, ('test' + str(imageNumber) + '-' + str(maxDepthRange) + '.png'))
    rawImage = cv2.imread(imagePath)
    # Prevent directly modifying original image
    image = cv2.cvtColor(rawImage, cv2.COLOR_BGR2GRAY)
    dummyImage = image.copy()

    # traverses through height of the image
    for i in range (dummyImage.shape[0]):
        # traverses through width of the image
        for j in range (dummyImage.shape[1]):
            # Since fence is always higher than kinect camera, we just take the upper depth map
            if (dummyImage[i][j] <= lowerColor):
                dummyImage[i][j] = 0

    rodCentreX = 0
    rodCount = 0
    # traverses through width of the image
    for j in range (dummyImage.shape[1]):
        count = 0
        # traverses height of first half of the image
        for i in range (dummyImage.shape[0]/2, 0, -1):
            if (dummyImage[i][j] > lowerColor):
                count += 1
            else:
                break
        if (count > 20):
            rodCentreX += j
            rodCount += 1

    if(rodCount > 0):
        rodCentreX = rodCentreX / rodCount

    cv2.line(rawImage, (rodCentreX, dummyImage.shape[0]/2), (rodCentreX, 0), (0, 255, 0), 2)

    listOfHeight = 0
    countList = 0
    blackZone = False
    circleY = 0
    # Finding y-coordinate of inner circle centre with known x-coordinate
    for i in range (dummyImage.shape[0]/2, 0, -1):
        count = 0
        for j, numpyArray in enumerate(dummyImage):
            for k, element in enumerate(numpyArray):
                if(dummyImage[i][j] > lowerColor):
                    dX = math.pow(k - rodCentreX, 2)
                    dY = math.pow(j - i, 2)
                    if(dX + dY < math.pow(15, 2)):
                        count += 1
        if (countList > 15):
            blackZone = True;
        if (count >= 10 and blackZone):
            break
        elif(count <= 10):
            listOfHeight += i
            countList += 1
    
    if(countList > 0):
        circleY = listOfHeight / countList

    print circleY
    
    # Finding radius of inner circle with known centre
    radius = 0
    for i in range (1, 50):
        count = 0
        for j, numpyArray in enumerate(dummyImage):
            for k, element in enumerate(numpyArray):
                if(dummyImage[i][j] > lowerColor):
                    dX = math.pow(k - rodCentreX, 2)
                    dY = math.pow(j - circleY, 2)
                    if(dX + dY > math.pow(i, 2)):
                        count += 1
        if (count > 10):
            break
        elif (count < 10 and radius < i):
            radius = i

    cv2.circle(rawImage, (rodCentreX, circleY), radius, (0, 0, 255), 2)

    cv2.imshow("output", rawImage)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


parsing()
colorCalculation()
imageProcess()