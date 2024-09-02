from PIL import Image
import os
from os import listdir




def main():
    # original size: 1920 × 1080 pixels
    width = 946
    height = 1079
    img_folder = "data_creation_process/coloscopy_101/images_751_754_2"
    center_width = 1130
    center_height = 540

    min_width = int(center_width - width/2)
    max_width = int(center_width + width/2)

    min_height = int(center_height - height/2)
    max_height = int(center_height + height/2)
    if not os.path.exists(os.path.join(img_folder,f"cw{center_width}_ch{center_height}_w{width}_h{height}")):
        os.mkdir(os.path.join(img_folder, f"cw{center_width}_ch{center_height}_w{width}_h{height}"))

    for images in os.listdir(img_folder):
        # check if the image ends with jpg
        if (images.endswith(".jpg")):
            img = Image.open(os.path.join(img_folder,images))
            im_crop = img.crop((min_width, min_height, max_width, max_height))
            im_crop.save(os.path.join(img_folder, f"cw{center_width}_ch{center_height}_w{width}_h{height}", images))




if __name__=="__main__": main()