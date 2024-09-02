import cv2
import numpy as np
import os

""" current image
################################ height_min
######             #############
#####               ############
###                  ########### height_corner(from center to point)
###                  ###########
###                  ###########
###                  ###########
###                  ###########
######              ############  height_square 
######             #############
################################ height_max
  width_min          width_max
      width_square
                   width_corner(from center to point)
"""
width = 946
height = 1079
img_folder = "data_creation_process/coloscopy_101/images_751_754_2/"
image = cv2.imread(os.path.join(img_folder,"23552.jpg"))
center_width = 1130
center_height = 540

min_width_corner = int(center_width - width/2)
max_width_corner = int(center_width + width/2)

min_height_corner = int(center_height - height/2)
max_height_corner = int(center_height + height/2)

corner_height = 200
corner_width = 200
square_height = 400
square_width = corner_width - 20

roi_corners = np.array([[(min_width_corner, min_height_corner), 
                         (max_width_corner, min_height_corner),
                         (max_width_corner + corner_width, min_height_corner + corner_height),
                         (max_width_corner + corner_width, max_height_corner - corner_height),
                         (max_width_corner, max_height_corner),
                         (min_width_corner - corner_width + square_width, max_height_corner),
                         (min_width_corner - corner_width + square_width, max_height_corner - square_height),
                         (min_width_corner - corner_width, max_height_corner - square_height),
                         (min_width_corner - corner_width, min_height_corner + corner_height)]])
img = cv2.imread
mask = np.zeros(image.shape, dtype=np.uint8)
channel_count = image.shape[2]  # i.e. 3 or 4 depending on your image
ignore_mask_color = (255,)*channel_count
cv2.fillConvexPoly(mask, roi_corners, ignore_mask_color)
# from Masterfool: use cv2.fillConvexPoly if you know it's convex

# apply the mask
masked_image = cv2.bitwise_and(image, mask)

# save the result
cv2.imwrite(os.path.join(img_folder,'image_masked.png'), masked_image)