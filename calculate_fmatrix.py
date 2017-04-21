import cv2
import sys
import numpy as np
import random
from matplotlib import pyplot as plt
from calibrate_camera import calibrate


def draw_points(img1, img2, pts1, pts2):
    """Draws the matching keypoints on the two images and numbers them
    for ease of checking

    Args:
        img1: First image
        img2: Second image
        pts1: Matched keypoints in first image
        pts2: Matched keypoints in second image
    Returns:
        None
    """
    img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    counter = 0
    for pt1, pt2 in zip(pts1, pts2):
        cv2.circle(img1, tuple(pt1), 5, 200, -1)
        cv2.circle(img2, tuple(pt2), 5, 200, -1)
        cv2.putText(img1, str(counter), tuple(pt1), cv2.FONT_HERSHEY_SIMPLEX,
                    1, 100, 2)
        cv2.putText(img2, str(counter), tuple(pt2), cv2.FONT_HERSHEY_SIMPLEX,
                    1, 100, 2)
        counter += 1

    return img1, img2


def check_solutions(fp, sp, K, R1, R2, t):
    """Using cv2.triangulatePoints to calculate position
    in real-world co-ordinate system. If z value for both
    points are positive, then that is our solution.
    More details can be found in the paper,
    David Nister, An efficient solution to the five-point relative pose problem.
    TODO: Need to take care of points that are far away

    Args:
        fp: Set of matched points from first image 
        sp: Set of matched points from second image
        K: Calibration matrix
        R1: First rotation matrix 
        R2: Second rotation matrix
        t: Translation vector
    Returns:
        R: Correct rotation matrix
    """
    dist = 50.0
    P0 = np.eye(3,4)
    P1 = np.dot(K, np.concatenate((R1, t.reshape(3,1)), axis=1))
    Q = cv2.triangulatePoints(P0, P1, fp.T, sp.T)
    mask1 = (Q[2, :] * Q[3, :]) > 0
    Q[0, :] /= Q[3, :]
    Q[1, :] /= Q[3, :]
    Q[2, :] /= Q[3, :]
    Q[3, :] /= Q[3, :]
    mask1 &= (Q[2,:] < dist)
    Q = np.dot(P1, Q)
    mask1 &= (Q[2,:] > 0)
    mask1 &= (Q[2,:] < dist)

    P2 = np.dot(K, np.concatenate((R2, t.reshape(3, 1)), axis=1))
    Q = cv2.triangulatePoints(P0, P2, fp.T, sp.T)
    mask2 = (Q[2, :] * Q[3, :]) > 0
    Q[0, :] /= Q[3, :]
    Q[1, :] /= Q[3, :]
    Q[2, :] /= Q[3, :]
    Q[3, :] /= Q[3, :]
    mask2 &= (Q[2, :] < dist)
    Q = np.dot(P1, Q)
    mask2 &= (Q[2, :] > 0)
    mask2 &= (Q[2, :] < dist)

    P3 = np.dot(K, np.concatenate((R1, -t.reshape(3, 1)), axis=1))
    Q = cv2.triangulatePoints(P0, P3, fp.T, sp.T)
    mask3 = (Q[2, :] * Q[3, :]) > 0
    Q[0, :] /= Q[3, :]
    Q[1, :] /= Q[3, :]
    Q[2, :] /= Q[3, :]
    Q[3, :] /= Q[3, :]
    mask3 &= (Q[2, :] < dist)
    Q = np.dot(P1, Q)
    mask3 &= (Q[2, :] > 0)
    mask3 &= (Q[2, :] < dist)

    P4 = np.dot(K, np.concatenate((R2, -t.reshape(3, 1)), axis=1))
    Q = cv2.triangulatePoints(P0, P4, fp.T, sp.T)
    mask4 = (Q[2, :] * Q[3, :]) > 0
    Q[0, :] /= Q[3, :]
    Q[1, :] /= Q[3, :]
    Q[2, :] /= Q[3, :]
    Q[3, :] /= Q[3, :]
    mask4 &= (Q[2, :] < dist)
    Q = np.dot(P1, Q)
    mask4 &= (Q[2, :] > 0)
    mask4 &= (Q[2, :] < dist)

    good1 = np.count_nonzero(mask1)
    good2 = np.count_nonzero(mask2)
    good3 = np.count_nonzero(mask3)
    good4 = np.count_nonzero(mask4)
    max_count = max(good1, good2, good3, good4)
    if max_count == good1:
        return R1
    elif max_count == good2:
        return R2
    elif max_count == good3:
        return R1
    elif max_count == good4:
        return R2

    return None


def calculate_fundamental_matrix(image1, image2, K,
                                 keypoint="SURF", outlier="LMEDS", draw_flag=False):
    """Calculation of Fundamental matrix from two images that have
    a valid epipolar geometry between them. Also decomposes the matrix to
    a Rotational matrix and Translation vector.
    TODO: Undistort points using distortion coefficient

    Args:
        image1: First image
        image2: Second image
        K: Calibration matrix
        keypoint: Keypoint detection method SIFT/SURF/ORB
        outlier: Outlier removal method RANSAC/LMEDS
        draw_flag: Flag to display matched checkpoints on two images
    Returns:
        f_matrix: Fundamental Matrix
        R: Rotational Matrix
        t: Translation vector
    """
    gray1 = cv2.imread(image1,cv2.CV_LOAD_IMAGE_GRAYSCALE)
    gray2 = cv2.imread(image2,cv2.CV_LOAD_IMAGE_GRAYSCALE)

    # FLANN parameters
    if keypoint == "SIFT":
        feature_detector = cv2.SIFT()
        index_params = dict(algorithm=0, trees=5)
    elif keypoint == "SURF":
        feature_detector = cv2.SURF()
        index_params = dict(algorithm=0, trees=5)
    elif keypoint == "ORB":
        feature_detector = cv2.ORB()
        index_params = dict(algorithm=6,
                            table_number=12,
                            key_size=12,
                            multi_probe_level=1)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    # Detecting keypoints in both images
    fkp, fkp_desc = feature_detector.detectAndCompute(gray1, None)
    skp, skp_desc = feature_detector.detectAndCompute(gray2, None)

    # Perform best matching between frames using nearest neighbors
    matches = flann.knnMatch(fkp_desc, skp_desc, k=2)
    matches = matches[:1000]

    pts1 = []
    pts2 = []
    # Lowe's ratio test
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            pts2.append(skp[m.trainIdx].pt)
            pts1.append(fkp[m.queryIdx].pt)

    pts1 = np.float32(pts1)
    pts2 = np.float32(pts2)
    if outlier == "RANSAC":
        f_matrix, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 1, 1.0)
    elif outlier == "LMEDS":
        f_matrix, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_LMEDS, 1, 1.0)

    # Selecting the inliers alone
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]

    # Draw matches
    if draw_flag:
        gray1 = cv2.imread(image1, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        gray2 = cv2.imread(image2, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        img1_n, img2_n = draw_points(gray1, gray2, pts1, pts2)
        plt.subplot(121), plt.imshow(img1_n)
        plt.subplot(122), plt.imshow(img2_n)
        plt.show()

    # Pick a random match, homogenize it and check if Fundamental
    # matrix calculated is good enough x' * F * x = 0
    index = random.randint(0, len(pts1))
    pt1 = np.array([[pts1[index][0], pts1[index][1], 1]])
    pt2 = np.array([[pts2[index][0], pts2[index][1], 1]])
    error = np.dot(pt1, np.dot(f_matrix, pt2.T))
    if error >= 1:
        print "Fundamental matrix calculation Failed! " + str(error)
        sys.exit()

    # This is the essential matrix
    E = K.T.dot(f_matrix).dot(K)

    # Decomposing the essential matrix
    U, S, Vt = np.linalg.svd(E)

    # http://isit.u-clermont1.fr/~ab/Classes/DIKU-3DCV2/Handouts/Lecture16.pdf
    # The two solutions possible for R are
    # R1 = UWVt or R2 = UWtVt
    W = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float32)
    R1 = U.dot(W).dot(Vt)
    R2 = U.dot(W.T).dot(Vt)
    t = U[:, 2]
    # OpenCV operates in the 4th quadrant i.e. x-axis increases to the right,
    # y-axis increases as it goes downwards and z-axis increases on line projected
    # from camera to object. So in our case, we want to find rotation about y-axis

    # Check which one of the 4 solutions are correct R1 t, R1 -t, R2 t, R2 -t
    # Hartley & Zisserman P 259-260 for Geometric interpretation of the 4 solutions
    R = check_solutions(pts1, pts2, K, R1, R2, t)

    if R is None:
        print "Calculation failed!"
        print "Possible Rs -> \n{}\n{}".format(R1, R2)
        return
    else:
        return f_matrix, R, t

if __name__ == "__main__":
    # Obtain camera intrinsics using camera_calibration.py
    images_directory = "../data/<calibration_directory>/*.jpg"
    r_error, d_coeff, K = calibrate(images_directory)

    image1 = "../data/<name>.jpg"
    image2 = "../data/<name>.jpg"

    f_matrix, R, t = calculate_fundamental_matrix(image1, image2, K)
