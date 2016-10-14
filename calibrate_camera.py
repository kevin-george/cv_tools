#NOTES
#Resize the image to 1000x1000 if the the calibration routine is taking too long
#The background to the chessboard needs to be white so corners can be detected easily
#The chessboard needs to be asymmetric i.e. number of rows should not be equal to number of columns
#The cornerSubPix function may make things worse and cause drawChessboardSquares to stop functioning

import cv2
import numpy as np

def calibrate(images):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    #Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((7*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)

    #Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    for entry in images:
        color = images[entry]
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        #Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,7), None)

        #If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            cv2.cornerSubPix(gray,corners,(4,4),(-1,-1),criteria)
            imgpoints.append(corners)

            #Draw and display the corners
            cv2.drawChessboardCorners(color, (9,7), corners, ret)
            cv2.imshow(entry, color)
            cv2.waitKey(0)


    image_size = gray.shape[::-1]
    reproj_err, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,
                                                              imgpoints,
                                                              image_size,
                                                              None, None)
    return reproj_err, mtx
