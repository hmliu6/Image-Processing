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
    print upperDistance, lowerDistance

def imageProcess():
    # Read image from ImageTest folder
    imagePath = os.path.join(imageDir, ('test' + str(imageNumber) + '-' + str(maxDepthRange) + '.png'))
    rawImage = cv2.imread(imagePath)
    # Prevent directly modifying original image
    image = cv2.cvtColor(rawImage, cv2.COLOR_BGR2GRAY)

    frontI = 0
    frontJ = 0
    backI = 0
    backJ = 0
    lineThickness = 2
    tempArray = []
    firstWhite = False
    changedFront = False
    changedBack = False
    # cv2.line(image, (frontJ, frontI), (j, i), (255, 255, 255), lineThickness)
    for i in range (image.shape[0]): # traverses through height of the image
        if(tempArray):
            tempArray.remove(tempArray[0])
            tempArray.remove(tempArray[0])
        if (frontI != 0 and frontJ != 0):
            tempArray.append((frontI, frontJ))
        if (backI != 0 and backJ != 0):
            tempArray.append((backI, backJ))
        print tempArray
        for j in range (image.shape[1]): # traverses through width of the image
            # Since fence is always higher than kinect camera, we just take the upper depth map
            if (i >= image.shape[0]/2):
                image[i][j] = 0
            elif (image[i][j] <= lowerColor):
                image[i][j] = 0
            elif (image[i][j] >= upperColor):
                image[i][j] = 0
            else:
                image[i][j] = 255
                if (firstWhite == False):
                    frontI = i
                    frontJ = j
                    backI = i
                    backJ = j
                elif (firstWhite == True and j < frontJ):
                    changedFront = True
                    frontI = i
                    frontJ = j
                elif (firstWhite == True and j > backJ):
                    changedBack = True
                    backI = i
                    backJ = j
                firstWhite = True
        if(tempArray):
            if (changedFront == True):
                cv2.line(image, (frontJ, frontI), (tempArray[0][1], tempArray[0][0]), (255, 255, 255), lineThickness)
                changedFront = False
            elif (changedBack == True):
                cv2.line(image, (backJ, backI), (tempArray[1][1], tempArray[1][0]), (255, 255, 255), lineThickness)
                changedBack = False

    # detect circles in the image
    circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1.2, 100)
    
    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
    
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
    
        # show the output image
        cv2.imshow("output", np.hstack([image, output]))
        cv2.waitKey(0)

    cv2.imshow('image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


parsing()
colorCalculation()
imageProcess()