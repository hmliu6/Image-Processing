import cv2
import numpy as np

cannyLower = 10
cannyUpper = 150
medianBlurValue = 5
detected = 0
points = []

for i in range(70):
# while(1):

    filePath = "throwing-ball/image" + str(1 + i) + ".png"
    print filePath
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

    (_, cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]

    detected -= 1

    for index, c in enumerate(cnts):
        if(len(c) > 150):
            continue
        
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        detected = 1
        points.append((cx, cy))
        print 'Centroid x: ' + str(cx) + ', y: ' + str(cy)
        cv2.line(image, (cx - 5, cy), (cx + 5, cy), (0, 255, 255), 2)
        cv2.line(image, (cx, cy - 5), (cx, cy + 5), (0, 255, 255), 2)

        for i, point in enumerate(points):
            if (i == 0):
                continue
            else:
                cv2.line(image, (points[i-1][0], points[i-1][1]), (point[0], point[1]), (255, 255, 0), 2)

        ellipse = cv2.fitEllipse(c)
        # cv2.ellipse(image, ellipse, (0, 0, 255), 2)

        (xC,yC),radius = cv2.minEnclosingCircle(c)
        center = (int(xC),int(yC))
        radius = int(radius)
        # cv2.circle(image, center, radius,(0, 255, 0), 2)

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        (x, y, w, h) = cv2.boundingRect(approx)
        area = cv2.contourArea(c)
        # print "index: {}, original: {}, approx: {}, area: {}".format(index, len(c), len(approx), str(area))
        # cv2.drawContours(image, [approx], -1, (255, 255, 0), 2)
        # cv2.putText(image, str(index), (x+(w/2), y+(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (4, 4, 253), 2)

    if(detected == -3):
        del points[:]

    cv2.imshow("Output", np.hstack((image, final0)))
    k = cv2.waitKey(0) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()