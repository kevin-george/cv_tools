import cv2
import sys


def capture_frames(video):
    """
        This method captures all frames from a video and writes it to the current
         directory
     """
    vid = cv2.VideoCapture(video)
    frame_num = 0
    while (vid.isOpened()):
        ret_val, frame = vid.read()
        if not (ret_val):
            break

        cv2.imwrite(str(frame_num) + ".jpg", frame)
        frame_num += 1
        if frame_num % 100 == 0:
            print "Captured " + str(frame_num) + " frames."

    vid.release()
    if frame_num == 0:
        print "The video is invalid"
    else:
        print "\nCaptured " + str(frame_num) + " frames and wrote to current directory"


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Script usage: "
        print "python capture_frames_from_video.py <video>"
        sys.exit()

    capture_frames(sys.argv[1])