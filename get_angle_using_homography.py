import cv2
import numpy as np
from matplotlib import pyplot as plt

class DriftDetector:
    def __init__(self, cam_intrinsics):
        #FLANN parameters
        FLANN_INDEX_KDTREE = 0
        #index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        index_params= dict(algorithm = 6,
                           table_number = 12,
                           key_size = 12,     # 20
                           multi_probe_level = 2)
        search_params = dict(checks = 5)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        #opencv2.4.x
        self.sift = cv2.ORB()
        #opencv3.x
        #self.sift = cv2.xfeatures2d.SIFT_create()

        self.K = cam_intrinsics
        self.K_inv = np.linalg.inv(cam_intrinsics)

    def drawpoints(self,img1,img2,pts1,pts2):
        img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
        img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
        counter = 0
        for pt1,pt2 in zip(pts1,pts2):
            cv2.circle(img1,tuple(pt1),5,200,-1)
            cv2.circle(img2,tuple(pt2),5,200,-1)
            cv2.putText(img1, str(counter), tuple(pt1), cv2.FONT_HERSHEY_SIMPLEX,
                        1, 100, 2)
            cv2.putText(img2, str(counter), tuple(pt2), cv2.FONT_HERSHEY_SIMPLEX,
                        1, 100, 2)
            counter += 1
        print counter

        return img1,img2

    def check_direction(self, first_points, second_points, rot, trans):
        for first, second in zip(first_points, second_points):
            first_z = np.dot(rot[0, :] - second[0]*rot[2, :], trans) / \
                        np.dot(rot[0, :] - second[0]*rot[2, :], second)

            first_3d_point = np.array([first[0] * first_z,
                                       first[1] * first_z, first_z])
            second_3d_point = np.dot(rot.T, first_3d_point) - np.dot(rot.T, trans)

            if first_3d_point[2] < 0 or second_3d_point[2] < 0:
                return False

            return True

    def opposites_of_minors(self, M, row, col):
        x1 = 1 if col == 0 else 0
        x2 = 1 if col == 2 else 2
        y1 = 1 if row == 0 else 0
        y2 = 1 if row == 2 else 2

        return M[y1, x2] * M[y2, x1] - M[y1, x1] * M[y2, x2]

    def findRmatFrom_tstar_n(self, H, tstar, n, v):
        #computes R = H( I - (2/v)*te_star*ne_t )
        tstar_m = np.array(tstar).reshape(3,1)
        n_m = np.array(n).reshape(1,3)
        I = np.identity(3)
        R = H.dot(I - (2/v) * (tstar_m.dot(n_m)))

        return R

    def decompose_homography(self, G):
        epsilon = 0.001

        #Normalize the Homography i.e. K^-1 * G * K
        H_hat = self.K_inv.dot(G).dot(self.K)

        #Remove Scale i.e. gamma = med(svd(H_hat))
        W = np.linalg.svd(H_hat, compute_uv = False)
        H = H_hat * (1.0/W[1])

        #S = H_trans * H - I
        S = H.T.dot(H)
        S[0, 0] -= 1.0
        S[1, 1] -= 1.0
        S[2, 2] -= 1.0

        #Check if H is a rotation matrix
        if(cv2.norm(S, cv2.NORM_INF) < epsilon):
            print "H is a rotation matrix"

        #M00, M11, M22
        M00 = self.opposites_of_minors(S, 0, 0)
        nS00 = np.abs(S[0,0]);
        M11 = self.opposites_of_minors(S, 1, 1)
        nS11 = np.abs(S[1,1]);
        M22 = self.opposites_of_minors(S, 2, 2)
        nS22 = np.abs(S[2,2]);
        indx = np.argmax([nS00, nS11, nS22])
        rtM00 = np.sqrt(M00)
        rtM11 = np.sqrt(M11)
        rtM22 = np.sqrt(M22)

        #M01, M12, M02
        M01 = self.opposites_of_minors(S, 0, 1)
        e01 = 1 if M01 >= 0 else -1
        M12 = self.opposites_of_minors(S, 1, 2)
        e12 = 1 if M12 >= 0 else -1
        M02 = self.opposites_of_minors(S, 0, 2)
        e02 = 1 if M02 >= 0 else -1

        npa = np.zeros(3)
        npb = np.zeros(3)
        if indx == 0:
            npa[0] = npb[0] = S[0,0]
            npa[1] = S[0,1] + rtM22
            npb[1] = S[0,1] - rtM22
            npa[2] = S[0,2] + e12 * rtM11
            npb[2] = S[0,2] - e12 * rtM11
        elif indx == 1:
            npa[0] = S[0,1] + rtM22
            npb[0] = S[0,1] - rtM22
            npa[1] = npb[1] = S[1,1]
            npa[2] = S[1,2] - e02 * rtM00
            npb[2] = S[1,2] + e02 * rtM00
        elif indx == 2:
            npa[0] = S[0,2] + e01 * rtM11
            npb[0] = S[0,2] - e01 * rtM11
            npa[1] = S[1,2] + rtM00
            npb[1] = S[1,2] - rtM00
            npa[2] = npb[2] = S[2,2]

        traceS = S[0,0] + S[1,1] + S[2,2]
        v = 2.0 * np.sqrt(1 + traceS - M00 - M11 - M22)

        ESii = 1 if S[indx,indx] >= 0 else -1
        r_2 = 2 + traceS + v
        nt_2 = 2 + traceS - v

        r = np.sqrt(r_2)
        n_t = np.sqrt(nt_2)

        na = npa / cv2.norm(npa);
        nb = npb / cv2.norm(npb);

        half_nt = 0.5 * n_t
        esii_t_r = ESii * r

        ta_star = half_nt * (esii_t_r * nb - n_t * na)
        tb_star = half_nt * (esii_t_r * na - n_t * nb)

        solutions = []
        #Ra, ta
        R = self.findRmatFrom_tstar_n(H, ta_star, na, v)
        t = R.dot(ta_star)
        solutions.append((R, t))
        #Ra, -ta
        solutions.append((R, -t))
        #Rb, tb
        R = self.findRmatFrom_tstar_n(H, tb_star, nb, v)
        t = R.dot(tb_star)
        solutions.append((R, t))
        #Rb, -tb
        solutions.append((R, -t))

        return solutions

    def find_correct_solution(self, solutions, first_set, second_set):
        for R, t in solutions:
            if not self.check_direction(first_set, second_set, R, t):
                break

        return R, t

    def get_homography_and_matches(self, first_kp, first_kp_desc,
                                   second_kp, second_kp_desc):
        #Perform best matching between frames using nearest neighbors
        matches = self.flann.knnMatch(first_kp_desc, second_kp_desc, k = 2)

        pts1 = []
        pts2 = []
        #Lowe's ratio test for outlier detection
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                pts2.append(second_kp[m.trainIdx].pt)
                pts1.append(first_kp[m.queryIdx].pt)

        #Compute the Homography
        pts1 = np.float32(pts1)
        pts2 = np.float32(pts2)
        G, mask = cv2.findHomography(pts1, pts2, cv2.LMEDS)

        #Selecting the inliers alone
        pts1 = pts1[mask.ravel() == 1]
        pts2 = pts2[mask.ravel() == 1]

        '''#Draw matches
        img1,img2 = self.drawpoints(gray1, gray2, pts1, pts2)
        plt.subplot(121),plt.imshow(img1)
        plt.subplot(122),plt.imshow(img2)
        plt.show()'''

        #Normalize and homogenize the image coordinates
        first_set = []
        second_set = []
        for i in range(len(pts1)):
            first_set.append(self.K_inv.dot([pts1[i][0], pts1[i][1], 1.0]))
            second_set.append(self.K_inv.dot([pts2[i][0], pts2[i][1], 1.0]))

        return G, first_set, second_set

    def get_drift(self, first_kp, first_kp_desc, second_kp, second_kp_desc):
        #Find the homography between the pairs of points and the matches that fit
        G, first_set, second_set = self.get_homography_and_matches(first_kp,
                                                                   first_kp_desc,
                                                                   second_kp,
                                                                   second_kp_desc)
        solutions = self.decompose_homography(G)
        R, t = self.find_correct_solution(solutions, first_set, second_set)
        yaw = np.arctan2(R[1][2], R[2][2]) * 180/3.1415
        pitch = np.arctan2(-R[2][0],
                           np.sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2])) * 180/3.1415
        roll = np.arctan2(R[1][0], R[0][0]) * 180/3.1415

        '''for R, t in solutions:
            yaw = np.arctan2(R[1][2], R[2][2]) * 180/3.1415
            pitch = np.arctan2(-R[2][0],
                           np.sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2])) * 180/3.1415
            roll = np.arctan2(R[1][0], R[0][0]) * 180/3.1415

            print "\nRoll: %f, Pitch: %f, Yaw: %f" %(roll , pitch , yaw)'''

        return roll, pitch, yaw

    def analyze_frame(self, frame):
        return self.sift.detectAndCompute(frame, None)


if __name__ == "__main__":
    #Camera intrinsices as obtained from the calibration modulea
    cam_intrinsics = np.float32([[733.64357007, 0, 373.41222549],
                                 [0, 788.17218714, 495.25800414],
                                 [0, 0, 1]])

    instance = DriftDetector(cam_intrinsics)
    img1 = "test_set/IMG_280.jpg"
    img2 = "test_set/IMG_275.jpg"
    gray1 = cv2.imread(img1,cv2.CV_LOAD_IMAGE_GRAYSCALE)
    gray2 = cv2.imread(img2,cv2.CV_LOAD_IMAGE_GRAYSCALE)
    print "\nComparing " + img1 + " and " + img2
    fkp, fkpd = instance.analyze_frame(gray1)
    skp, skpd = instance.analyze_frame(gray2)
    r,p,y = instance.get_drift(fkp, fkpd, skp, skpd)
    print "\n\nRoll: %f, Pitch: %f, Yaw: %f" %(r, p, y)

    '''#Specifying the video to be captured
    vid = cv2.VideoCapture('VID_20160831_105333.mp4')
    #In case of video stream, use this
    #vid = cv2.VideoCapture(0)
    while(vid.isOpened()):
        #Reading the frame from the video
        ret_val, frame = vid.read()
        if not(ret_val):
            break
        #Converting the frame to grayscale
        if counter == 0:
            gray1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detector = DriftDetector(cam_intrinsics)
            first_kp, first_kp_desc = detector.analyze_frame(gray1)
        elif counter == 68:
            gray2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            second_kp, second_kp_desc = detector.analyze_frame(gray2)
            roll, pitch, yaw = detector.get_drift(first_kp, first_kp_desc,
                                                  second_kp, second_kp_desc,
                                                  gray1, gray2)
            print "\n\nRoll: %f, Pitch: %f, Yaw: %f" %(roll , pitch , yaw)
            #Show images being compared
            plt.subplot(121),plt.imshow(gray1)
            plt.subplot(122),plt.imshow(gray2)
            plt.show()
            break
        counter += 1
    vid.release()'''
