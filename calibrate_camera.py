"""
NOTES
1. Resize the image to 1000x1000 if the the calibration routine is taking too long
2. The background to the chessboard needs to be white so corners can be detected easily
3. The chessboard needs to be asymmetric i.e. number of rows should not be equal to number of columns
4. The cornerSubPix function may make things worse and cause drawChessboardSquares to stop functioning
"""

import cv2
import numpy as np
from glob import glob


def calibrate(images_directory, display=False, verbose=False):
    """
    This method takes a set of images of an asymmetric chessboard and
    returns the camera intrinsic matrix
    """
    criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((7*9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:7].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    gray = None
    for file_name in glob(images_directory + "/*"):
        try:
            if verbose:
                print "Reading file {}".format(file_name)
            color = cv2.imread(file_name)
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

            # Find the chess board grid
            ret, corners = cv2.findChessboardCorners(gray, (9,7), None)

            # If found, add object points, image points (after refining them)
            if ret:
                if verbose:
                    print "Chessboard grid detected for {}".format(file_name)
                objpoints.append(objp)

                cv2.cornerSubPix(gray,corners,(4,4),(-1,-1),criteria)
                imgpoints.append(corners)

                # Draw and display the corners
                if display:
                    cv2.drawChessboardCorners(color, (9,7), corners, ret)
                    cv2.imshow(file_name, color)
                    cv2.waitKey(0)
        except Exception as e:
            if verbose:
                print "Exception was caught {}".format(e)
            return 0

    if not gray is None:
        image_size = gray.shape[::-1]
        reproj_err, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
                                                                  image_size, None, None)
        if verbose:
            print "\nCamera Intrinsic Matrix(K) ->\n{}".format(mtx)
            print "\nRe-projection Error -> {}".format(reproj_err)

        if reproj_err >= 0.5:
            print "\n\nRe-projection Error is too high! Please check chessboard detection using the display flag"
            print "python calibrate_camera.py <directory_of_images> --display"
        else:
            return reproj_err, mtx, dist, rvecs, tvecs

    return 0


if __name__ == "__main__":
    calibrate("./images", verbose=True)