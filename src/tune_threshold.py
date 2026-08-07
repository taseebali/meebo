
import sys
import cv2
import numpy as np

# Deliberately generic starting bounds — a rough "yellow-ish" range,
# unlikely to exactly match your track's lane color or your camera's
# color response. Replace with the numbers you measured in step 3.
HSV_LOWER = np.array([30, 2, 0])
HSV_UPPER = np.array([90, 22, 110])

# TODO: crop to the band of the frame where the lane actually appears,
# using what you noted in step 2 and the frame height from step 1.
# ROI_TOP/ROI_BOTTOM are row (y) pixel bounds.
ROI_TOP = 360
ROI_BOTTOM = 800   # None = no crop yet — set a real value once you know your frame height

path = sys.argv[1] if len(sys.argv) > 1 else 'frame_000.png'
frame = cv2.imread(path)

roi = frame[ROI_TOP:ROI_BOTTOM, :]
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

out_path = path.replace('.png', '_mask.png')
cv2.imwrite(out_path, mask)
print(f'Wrote {out_path}  lane pixels found: {int((mask > 0).sum())}')
