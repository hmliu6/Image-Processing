import cv2
import numpy as np

cannyLower = 10
cannyUpper = 150
medianBlurValue = 5
updated = False

def do_nothing(x):
    return

def updateBlur(x):
    global medianBlurValue, updated
    medianBlurValue = x
    updated = True

def updateUpper(x):
    global cannyLower, updated
    cannyLower = x
    updated = True

def updateLower(x):
    global cannyLower, updated
    cannyLower = x
    updated = True

cv2.namedWindow('Output')
cv2.createTrackbar('Median Blur', 'Output', 0, 20, updateBlur)
cv2.createTrackbar('Upper Bound', 'Output', 0, 255, updateUpper)
cv2.createTrackbar('Lower Bound', 'Output', 0, 255, updateLower)

for i in range(40):
# while(1):
    k = cv2.waitKey(0) & 0xFF
    if k == 27:
        break

    filePath = "throwing-ball/image" + str(29 + i) + ".png"
    image = cv2.imread(filePath)

    # traverses through height of the image
    for i in range (image.shape[0]):
        # traverses through width of the image
        for j in range (image.shape[1]):
            # Since fence is always higher than kinect camera, we just take the upper depth map
            if (i >= image.shape[0] / 2):
                image[i][j] = (0, 0, 0)

    final0 = cv2.medianBlur(image, 2 * medianBlurValue + 1)
    gray = cv2.cvtColor(final0, cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, cannyLower, cannyUpper)
    edgedOutput = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)

    (_, cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]

    for index, c in enumerate(cnts):
        if(len(c) > 150):
            continue

        (xC,yC),radius = cv2.minEnclosingCircle(c)
        center = (int(xC),int(yC))
        radius = int(radius)
        cv2.circle(image, center, radius,(0, 255, 0), 2)

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        (x, y, w, h) = cv2.boundingRect(approx)
        area = cv2.contourArea(c)
        print "index: {}, original: {}, approx: {}, area: {}".format(index, len(c), len(approx), str(area))
        cv2.drawContours(image, [approx], -1, (255, 255, 0), 2)
        cv2.putText(image, str(index), (x+(w/2), y+(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (4, 4, 253), 2)

    cv2.imshow("Output", np.hstack((image, edgedOutput, final0)))

cv2.destroyAllWindows()