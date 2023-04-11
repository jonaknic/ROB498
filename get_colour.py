import cv2
import numpy as np
from scipy.spatial import distance as dist

def get_colour(img, contour=None):
    
    # returns 0 for red, 1 for green
    
    # img = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
    # red = np.array((255, 0, 0)).astype("uint8").reshape(1, 1, 3)
    # red = cv2.cvtColor(red, cv2.COLOR_RGB2LAB)
    # green = np.array((0, 255, 0)).astype("uint8").reshape(1, 1, 3)
    # green = cv2.cvtColor(green, cv2.COLOR_RGB2LAB)
    
    red = (255, 0, 0)
    green = (0, 255, 0)
    
    if contour:
        mask = np.zeros(img.shape[:2], dtype="uint8")
        cv2.drawContours(mask, [contour], -1, 255, -1)
        mask = cv2.erode(mask, None, iterations=2)
    else:
        mask = None
        
    mean = cv2.mean(img, mask=mask)[:3]
    
    d_red = dist.euclidean(red, mean)
    d_green = dist.euclidean(green, mean)
    
    return 0 if d_red <= d_green else 1
