import cv2
import numpy

sensitivity = 5
color_space = cv2.COLOR_BGR2HSV
step_size = 4

img = cv2.imread('middle.jpg')
rotoscope = cv2.imread('in1.png')
rotoscope = cv2.cvtColor(rotoscope, cv2.COLOR_BGR2GRAY)
image, contours, hierarchy = cv2.findContours(rotoscope, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

contour = contours[0]
M = cv2.moments(contour)
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])

print("Centroid (x):" + str(cx))
print("Centroid (y):" + str(cy))

converted = cv2.cvtColor(img, color_space)
print(str(converted[cx, cy]))
sample = converted[cx, cy]

def nothing(x):
    pass

cv2.namedWindow('Tuner', cv2.WINDOW_GUI_EXPANDED)
cv2.createTrackbar('a1', 'Tuner', 0, 255, nothing)
cv2.createTrackbar('a2', 'Tuner', 0, 255, nothing)
cv2.createTrackbar('b1', 'Tuner', 0, 255, nothing)
cv2.createTrackbar('b2', 'Tuner', 0, 255, nothing)
cv2.createTrackbar('c1', 'Tuner', 0, 255, nothing)
cv2.createTrackbar('c2', 'Tuner', 0, 255, nothing)
cv2.setTrackbarPos('a1', 'Tuner', sample[0] - sensitivity)
cv2.setTrackbarPos('a2', 'Tuner', sample[0] + sensitivity)
cv2.setTrackbarPos('b1', 'Tuner', sample[1] - sensitivity)
cv2.setTrackbarPos('b2', 'Tuner', sample[1] + sensitivity)
cv2.setTrackbarPos('c1', 'Tuner', sample[2] - sensitivity)
cv2.setTrackbarPos('c2', 'Tuner', sample[2] + sensitivity)
#cv2.createTrackbar('d1', 'Tuner', 0, 255, nothing)
#cv2.createTrackbar('d2', 'Tuner', 0, 255, nothing)

switch = 's'
cv2.createTrackbar(switch, 'Tuner', 0, 1, nothing)

while True:
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    a1 = cv2.getTrackbarPos('a1', 'Tuner')
    b1 = cv2.getTrackbarPos('a2', 'Tuner')
    a2 = cv2.getTrackbarPos('b1', 'Tuner')
    b2 = cv2.getTrackbarPos('b2', 'Tuner')
    a3 = cv2.getTrackbarPos('c1', 'Tuner')
    b3 = cv2.getTrackbarPos('c2', 'Tuner')
    #a4 = cv2.getTrackbarPos('d1', 'Tuner')
    #b4 = cv2.getTrackbarPos('d2', 'Tuner')
    s = cv2.getTrackbarPos(switch, 'Tuner')

    mask = cv2.inRange(converted, (a1, a2, a3), (b1, b2, b3))
    #secondary_mask = cv2.inRange(converted, (a4, a2, a3), (b4, b2, b3))

    #combined_mask = cv2.bitwise_or(mask, secondary_mask)
    if s is 0:
        cv2.imshow('Tuner', img)
    else:
        cv2.imshow('Tuner', mask)
