import numpy

import cv2

# Procedure
# for each channel
#
# increase lower bound
# stop once max amount is lost
#
# decrease upper bound
# stop once max amount is lost

step_size = 1
max_loss = 0.05

image = cv2.imread('input.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

mask = cv2.imread('mask.png')
mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
tmp, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


# True if amount lost exceeds limit
def test_amount_lost(upper, lower, channel, channels, contour_size):
    global max_loss

    lower_scalar = [0] * channels
    upper_scalar = [255] * channels

    lower_scalar[channel] = lower
    upper_scalar[channel] = upper

    lower_scalar = tuple(lower_scalar)
    upper_scalar = tuple(upper_scalar)

    test_mask = cv2.inRange(image, lower_scalar, upper_scalar)
    test_mask = cv2.bitwise_and(test_mask, mask)

    contours_sum = cv2.countNonZero(test_mask)
    return (contours_sum / contour_size) > (1 - max_loss / 2)


def optimize_channel(channel):
    lower = 0
    upper = 255

    contour_size = cv2.contourArea(contours[0])

    while test_amount_lost(255, lower, channel, 3, contour_size):
        if lower > 255:
            lower = 0
            break
        lower += step_size

    while test_amount_lost(upper, 0, channel, 3, contour_size):
        if upper < 0:
            upper = 255
            break
        upper -= step_size

    return lower, upper


h_lower, h_upper = optimize_channel(0)
s_lower, s_upper = optimize_channel(1)
v_lower, v_upper = optimize_channel(2)

print('output')
print((h_lower, s_lower, v_lower))
print((h_upper, s_upper, v_upper))
