import cv2
import numpy as np
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


def opposites_of_minors(M, row, col):
    x1 = 1 if col == 0 else 0
    x2 = 1 if col == 2 else 2
    y1 = 1 if row == 0 else 0
    y2 = 1 if row == 2 else 2

    return M[y1, x2] * M[y2, x1] - M[y1, x1] * M[y2, x2]


def findRmatFrom_tstar_n(H, tstar, n, v):
    # computes R = H( I - (2/v)*te_star*ne_t )
    tstar_m = np.array(tstar).reshape(3, 1)
    n_m = np.array(n).reshape(1, 3)
    I = np.identity(3)
    R = H.dot(I - (2 / v) * (tstar_m.dot(n_m)))

    return R


def decompose_homography(G, K):
    epsilon = 0.001

    # Normalize the Homography i.e. K^-1 * G * K
    K_inv = np.linalg.inv(K)
    H_hat = K_inv.dot(G).dot(K)

    # Remove Scale i.e. gamma = med(svd(H_hat))
    W = np.linalg.svd(H_hat, compute_uv=False)
    H = H_hat * (1.0 / W[1])

    # S = H_trans * H - I
    S = H.T.dot(H)
    S[0, 0] -= 1.0
    S[1, 1] -= 1.0
    S[2, 2] -= 1.0

    # Check if H is a rotation matrix
    if (cv2.norm(S, cv2.NORM_INF) < epsilon):
        print "H is a rotation matrix"

    # M00, M11, M22
    M00 = opposites_of_minors(S, 0, 0)
    nS00 = np.abs(S[0, 0]);
    M11 = opposites_of_minors(S, 1, 1)
    nS11 = np.abs(S[1, 1]);
    M22 = opposites_of_minors(S, 2, 2)
    nS22 = np.abs(S[2, 2]);
    indx = np.argmax([nS00, nS11, nS22])
    rtM00 = np.sqrt(M00)
    rtM11 = np.sqrt(M11)
    rtM22 = np.sqrt(M22)

    # M01, M12, M02
    M01 = opposites_of_minors(S, 0, 1)
    e01 = 1 if M01 >= 0 else -1
    M12 = opposites_of_minors(S, 1, 2)
    e12 = 1 if M12 >= 0 else -1
    M02 = opposites_of_minors(S, 0, 2)
    e02 = 1 if M02 >= 0 else -1

    npa = np.zeros(3)
    npb = np.zeros(3)
    if indx == 0:
        npa[0] = npb[0] = S[0, 0]
        npa[1] = S[0, 1] + rtM22
        npb[1] = S[0, 1] - rtM22
        npa[2] = S[0, 2] + e12 * rtM11
        npb[2] = S[0, 2] - e12 * rtM11
    elif indx == 1:
        npa[0] = S[0, 1] + rtM22
        npb[0] = S[0, 1] - rtM22
        npa[1] = npb[1] = S[1, 1]
        npa[2] = S[1, 2] - e02 * rtM00
        npb[2] = S[1, 2] + e02 * rtM00
    elif indx == 2:
        npa[0] = S[0, 2] + e01 * rtM11
        npb[0] = S[0, 2] - e01 * rtM11
        npa[1] = S[1, 2] + rtM00
        npb[1] = S[1, 2] - rtM00
        npa[2] = npb[2] = S[2, 2]

    traceS = S[0, 0] + S[1, 1] + S[2, 2]
    v = 2.0 * np.sqrt(1 + traceS - M00 - M11 - M22)

    ESii = 1 if S[indx, indx] >= 0 else -1
    r_2 = 2 + traceS + v
    nt_2 = 2 + traceS - v

    r = np.sqrt(r_2)
    n_t = np.sqrt(nt_2)

    na = npa / cv2.norm(npa)
    nb = npb / cv2.norm(npb)

    half_nt = 0.5 * n_t
    esii_t_r = ESii * r

    ta_star = half_nt * (esii_t_r * nb - n_t * na)
    tb_star = half_nt * (esii_t_r * na - n_t * nb)

    # TO-DO: Figure out what's up with these solutions
    """solutions = []
    # Ra, ta
    R = findRmatFrom_tstar_n(H, ta_star, na, v)
    t = R.dot(ta_star)
    solutions.append((R, t, na))
    # Ra, -ta
    solutions.append((R, -t, -na))
    # Rb, tb
    R = findRmatFrom_tstar_n(H, tb_star, nb, v)
    t = R.dot(tb_star)
    solutions.append((R, t, nb))
    # Rb, -tb
    solutions.append((R, -t, -nb))

    return solutions"""

    R1 = findRmatFrom_tstar_n(H, ta_star, na, v)
    t = R1.dot(ta_star)
    R2 = findRmatFrom_tstar_n(H, tb_star, nb, v)
    return R1, R2, t

def calculate_homography(image1, image2, K,
                         keypoint="SURF", outlier="LMEDS", draw_flag=False):
    """Calculation of Homography from two images, Also decomposes the Homography to
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
        G: Homography
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
        G, mask = cv2.findHomography(pts1, pts2, cv2.RANSAC, 1.0)
    elif outlier == "LMEDS":
        G, mask = cv2.findHomography(pts1, pts2, cv2.LMEDS)

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

    tot_err = 0
    for i in range(len(pts1)):
        calc_pt = G.dot(np.float32(pts1[i]).T)
        err = np.sqrt(np.square(calc_pt[0] - pts2[i][0]) \
                      + np.square(calc_pt[1] - pts2[i][1]))
        tot_err += err
    print "\nAverage Reprojection error: " + str(tot_err / len(pts1))

    R1, R2, t = decompose_homography(G, K)
    R, t = check_solutions(pts1, pts2, K, R1, R2, t)

    return G, R, t


if __name__ == "__main__":
    # Obtain camera intrinsics using camera_calibration.py
    images_directory = "../data/<calibration_directory>/*.jpg"
    r_error, d_coeff, K = calibrate(images_directory)

    image1 = "../data/<name>.jpg"
    image2 = "../data/<name>.jpg"

    G, R, t = calculate_homography(image1, image2, K)
