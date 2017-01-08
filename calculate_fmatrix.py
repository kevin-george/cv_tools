import cv2
import numpy as np
from matplotlib import pyplot as plt

class Vision:
    def __init__(self, mni):
        self.mni = mni

        #FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 5)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.sift = cv2.SIFT()
        self.source_kp, self.source_kp_desc = self.sift.detectAndCompute(mni, None)

    def drawlines(self,img1,img2,lines,pts1,pts2):
        r,c = img1.shape
        img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
        img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
        counter = 0
        for r,pt1,pt2 in zip(lines,pts1,pts2):
            color = tuple(np.random.randint(0,255,3).tolist())
            x0,y0 = map(int, [0, -r[2]/r[1] ])
            x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
            cv2.line(img1, (x0,y0), (x1,y1), color,1)
            cv2.circle(img1,tuple(pt1),5,color,-1)
            cv2.circle(img2,tuple(pt2),5,color,-1)
            cv2.putText(img1, str(counter), tuple(pt1), cv2.FONT_HERSHEY_SIMPLEX, 1, 100, 2)
            counter += 1

        return img1,img2

    def check_direction(self, first_points, second_points, rot, trans):
        for first, second in zip(first_points, second_points):
            first_z = np.dot(rot[0, :] - second[0]*rot[2, :], trans) / \
                      np.dot(rot[0, :] - second[0]*rot[2, :], second)

            first_3d_point = np.array([first[0] * first_z, first[1] * first_z, first_z])
            second_3d_point = np.dot(rot.T, first_3d_point) - np.dot(rot.T, trans)

            if first_3d_point[2] < 0 or second_3d_point[2] < 0:
                return False

        return True

    def get_drift(self, estimated_mni):
        pts1 = []
        pts2 = []

        #Perform best matching between frames using nearest neighbors
        dest_kp, dest_kp_desc = self.sift.detectAndCompute(estimated_mni, None)
        matches = self.flann.knnMatch(self.source_kp_desc, dest_kp_desc, k = 2)

        #Lowe's ratio test for outlier detection
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                pts2.append(dest_kp[m.trainIdx].pt)
                pts1.append(self.source_kp[m.queryIdx].pt)

        pts1 = np.float32(pts1)
        pts2 = np.float32(pts2)
        F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_LMEDS, 1, 1.0)

        #Selecting the inliers alone
        pts1 = pts1[mask.ravel() == 1]
        pts2 = pts2[mask.ravel() == 1]

        # Find epilines corresponding to points in right image (second image) and
        # drawing its lines on left image
        lines1 = cv2.computeCorrespondEpilines(pts2.reshape(-1,1,2), 2,F)
        lines1 = lines1.reshape(-1,3)
        img5,img6 = self.drawlines(self.mni,estimated_mni,lines1,pts1,pts2)

        # Find epilines corresponding to points in left image (first image) and
        # drawing its lines on right image
        lines2 = cv2.computeCorrespondEpilines(pts1.reshape(-1,1,2), 1,F)
        lines2 = lines2.reshape(-1,3)
        img3,img4 = self.drawlines(estimated_mni,self.mni,lines2,pts2,pts1)

        #Plot the epipolar linese to see if they make sense
        plt.subplot(121),plt.imshow(img5)
        plt.subplot(122),plt.imshow(img3)
        plt.show()

        '''#Homogenous co-ordinates are created from the inliers
        pt1 = np.array([[pts1[0][0]], [pts1[0][1]], [1]])
        pt2 = np.array([[pts2[0][0], pts2[0][1], 1]])

        print "Fundamental matrix error check: %f"%np.dot(np.dot(pt2,F),pt1)'''

        #Camera intrinsics generated from calibrate_camera.py
        K = np.float32([[733.64357007, 0, 373.41222549],
                       [0, 788.17218714, 495.25800414],
                       [0, 0, 1]])
        K_inv = np.linalg.inv(K)

        #This is the essential matrix
        E = K.T.dot(F).dot(K)

        U, S, Vt = np.linalg.svd(E)
        #Taken from Hartley and Zisserman
        W = np.array([0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]).reshape(3, 3)

        #Normalize and homogenize the image co-ordinates
        first_inliers = []
        second_inliers = []
        for i in range(len(pts1)):
            first_inliers.append(K_inv.dot([pts1[i][0], pts1[i][1], 1.0]))
            second_inliers.append(K_inv.dot([pts2[i][0], pts2[i][1], 1.0]))

        #Checking the four possible solutions to the decomposition
        #R = U * W * Vt, T = u_3
        R = U.dot(W).dot(Vt)
        T = U[:, 2]
        #print "First"
        if not self.check_direction(first_inliers, second_inliers, R, T):
            #R = U * W * Vt, T = -u_3
            T = - U[:, 2]
            #print "Second"
            if not self.check_direction(first_inliers, second_inliers, R, T):
                #R = U * Wt * Vt, T = u_3
                R = U.dot(W.T).dot(Vt)
                T = U[:, 2]
                #print "Third"
                if not self.check_direction(first_inliers, second_inliers, R, T):
                    #R = U * Wt * Vt, T = -u_3
                    T = - U[:, 2]
                    #print "Fourth"

        #print "\n Rotational Matrix \n" + str(R)
        #print "\n Translational Vector " + str(T)

        yaw = np.arctan2(R[1][2], R[2][2]) * 180/3.1415
        pitch = np.arctan2(-R[2][0], np.sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2])) * 180/3.1415
        roll = np.arctan2(R[1][0],  R[0][0]) * 180/3.1415

        #print "\n Roll: %f, Pitch: %f, Yaw: %f" %(roll , pitch , yaw)


if __name__ == "__main__":
    #Specifying the video to be captured
    vid = cv2.VideoCapture('VID_20160831_105333.mp4')
    #In case of video stream, use this
    #vid = cv2.VideoCapture(0)
    counter = 0
    '''img1 = "test_set/IMG_270.jpg"
    img2 = "test_set/IMG_280.jpg"
    gray1 = cv2.imread(img1,cv2.CV_LOAD_IMAGE_GRAYSCALE)
    gray2 = cv2.imread(img2,cv2.CV_LOAD_IMAGE_GRAYSCALE)
    v = Vision(gray1)
    v.get_drift(gray2)'''

    while(vid.isOpened()):
        #Reading the frame from the video
        ret_val, frame = vid.read()
        if not(ret_val):
            break

        #Converting the frame to grayscale
        if counter == 0:
            gray1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            v = Vision(gray1)
        elif counter == 65:
            gray2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            v.get_drift(gray2)
            #Show images being compared
            '''plt.subplot(121),plt.imshow(gray1)
            plt.subplot(122),plt.imshow(gray2)
            plt.show()'''

            break

        counter += 1

    vid.release()
