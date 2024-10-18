import cv2
import os


def main():
    directory = "coloscopy_102"
    video_name = '102_Endo_Colo_HD.mp4'
    down_sample = 5

    # if only part of the video should be processed
    only_part_video = True
    fps = 50
    min_s = 11 * 60 + 44
    total_s = 12 * 60 + 0
    

    current_dir = os.getcwd()
    video_path = os.path.join(current_dir, 'data_creation_process', directory, video_name)
    image_path = os.path.join(current_dir, 'data_creation_process', directory, "images")
    
    # create images directory
    if not os.path.exists(image_path):
        os.mkdir(image_path)
    
    # start video Capture tool
    cap = cv2.VideoCapture(video_path)
    frame_no = 0
    print('start')
    while(cap.isOpened()):
        # capture frame
        ret, frame = cap.read()

        print(frame_no)
        # set frame no up
        frame_no += 1
        if only_part_video:   
            # skip first part of the video
            if frame_no < fps * min_s:
                continue

        if frame_no % down_sample == 0 and ret:
            # save frame as image in images
            target = str(os.path.join(image_path,f"{frame_no}.jpg"))
            cv2.imwrite(target, frame)

        

        if only_part_video:
            # skip last part of the video
            if frame_no > fps * total_s:
                break

    cap.release()
    print('done')


if __name__=="__main__": main()