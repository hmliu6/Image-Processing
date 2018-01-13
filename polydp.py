import cv2
import numpy as np

for i in range(20):
    filePath = "throwing-ball/image" + str(29 + i) + ".png"
    # filePath = "highestRod/test" + str(1 + i) + "-8000.png"
    image = cv2.imread(filePath)
    final0 = cv2.medianBlur(image, 19)
    gray = cv2.cvtColor(final0, cv2.COLOR_BGR2GRAY)
    edged = cv2.Canny(gray, 10, 150)

    # cv2.imshow("Median Blur", final0)

    (_, cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:7]

    for index, c in enumerate(cnts):
        if(len(c) > 100):
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        (x, y, w, h) = cv2.boundingRect(approx)
        print "index: {}, original: {}, approx: {}".format(index, len(c), len(approx))
        cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
        cv2.putText(image, str(index), (x+(w/2), y+(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (4, 4, 253), 2)

    cv2.imshow("Output", np.hstack((image, final0)))
    cv2.waitKey(0)