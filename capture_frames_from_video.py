import cv2


def capture_frames(video_file, image_format="jpg", verbose=False):
    """This method captures all frames from a video and writes it to the current
    directory

    Args:
        video_file: The video that needs to be processed
        image_format: The format of the images to be saved
        verbose: Flag to print debug statements
    Returns:
        None
    """
    vid = cv2.VideoCapture(video_file)
    frame_num = 0
    while (vid.isOpened()):
        ret_val, frame = vid.read()
        if not ret_val:
            break

        cv2.imwrite("{}.{}".format(frame_num,image_format), frame)
        frame_num += 1
        if frame_num % 100 == 0:
            if verbose:
                print "Captured {} frames".format(frame_num)

    vid.release()
    if frame_num == 0:
        print "The video is invalid!"
    else:
        print "\nCaptured {} frames and wrote to current directory".format(frame_num)


if __name__ == "__main__":
    capture_frames("./video.mp4", verbose=True)
