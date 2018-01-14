import cv2
import numpy as np

for i in range(40):
    filePath = "throwing-ball/image" + str(29 + i) + ".png"
    image = cv2.imread(filePath)

    # traverses through height of the image
    for i in range (image.shape[0]):
        # traverses through width of the image
        for j in range (image.shape[1]):
            # Since fence is always higher than kinect camera, we just take the upper depth map
            if (i >= image.shape[0] / 2):
                image[i][j] = (0, 0, 0)

    final0 = cv2.medianBlur(image, 13)
    gray = cv2.cvtColor(final0, cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, 10, 200)
    edgedOutput = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)

    # cv2.imshow("Median Blur", final0)

    (_, cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]

    for index, c in enumerate(cnts):
        if(len(c) > 100):
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        (x, y, w, h) = cv2.boundingRect(approx)
        print "index: {}, original: {}, approx: {}".format(index, len(c), len(approx))
        cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
        cv2.putText(image, str(index), (x+(w/2), y+(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (4, 4, 253), 2)

    cv2.imshow("Output", np.hstack((image, edgedOutput, final0)))
    cv2.waitKey(0)