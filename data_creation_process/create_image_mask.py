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
      <---->width-corner min
            <---->width-corner max 
   <-> square width
      corner width <->            
"""
width = 946
height = 1079
img_folder = "/home/yn86eniw/Documents/Bachelorarbeit2/data_creation_process/coloscopy_102/images"
image_path = os.path.join("/home/yn86eniw/Documents/Bachelorarbeit2/data_creation_process/coloscopy_102/images")
image = cv2.imread(os.path.join(img_folder,"35200.jpg"))

if not os.path.exists(os.path.join(image_path, "mask")):
    os.mkdir(os.path.join(image_path, "mask"))

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

mask = np.zeros(image.shape, dtype=np.uint8)
channel_count = image.shape[2]
ignore_mask_color = (255,)*channel_count
cv2.fillConvexPoly(mask, roi_corners, ignore_mask_color)

for images in os.listdir(img_folder):
        # check if the image ends with jpg
        if (images.endswith(".jpg")):
                cv2.imwrite(os.path.join(img_folder, "mask",f'{images}.png'), mask)
