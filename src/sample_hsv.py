import sys
import cv2

X1, Y1, X2, Y2 = 380, 450, 420, 500

path = sys.argv[1] if len(sys.argv) > 1 else 'frame_000.png'
frame = cv2.imread(path)

patch = frame[Y1:Y2, X1:X2]
hsv_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)

print('patch shape:', patch.shape)
print('H min/max:', hsv_patch[:, :, 0].min(), hsv_patch[:, :, 0].max())
print('S min/max:', hsv_patch[:, :, 1].min(), hsv_patch[:, :, 1].max())
print('V min/max:', hsv_patch[:, :, 2].min(), hsv_patch[:, :, 2].max())
