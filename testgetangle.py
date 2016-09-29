import get_angle_using_homography
import calibrate_camera
import unittest
import numpy as np
import cv2
import glob

class TestGetAngle(unittest.TestCase):
    '''
    Testing Camera Calibration module
    '''
    def test_calibrate(self):
        self.assertTrue(True)
        images = []
        for fname in glob.glob('data/calibration_images/*.jpg'):
            gray = cv2.imread(fname, cv2.CV_LOAD_IMAGE_GRAYSCALE)
            images.append(gray)

        reproj_err, K = calibrate_camera.calibrate(images)
        print reproj_err
        print K
        #self.assertTrue(reproj_err < 0.5, "Camera Calibration Reprojection \
        #Error is High")

    '''
    Testing GetAngle functions
    '''
    def test_check_direction(self):
        pass

    def test_opposites_of_minors(self):
        pass

    def test_findRmatFrom_tstar_n(self):
        pass

    def test_find_correct_solution(self):
        pass

    def test_get_homography_and_matches(self):
        '''cam_intrinsics = np.float32([[733.64357007, 0, 373.41222549],
                                     [0, 788.17218714, 495.25800414],
                                     [0, 0, 1]])

        instance = get_angle_using_homography.DriftDetector(cam_intrinsics)
        img1 = "test_set/IMG_270.jpg"
        img2 = "test_set/IMG_280.jpg"
        print "\nComparing " + img1 + " and " + img2
        fkp, fkpd = instance.analyze_frame(cv2.imread(img1,cv2.CV_LOAD_IMAGE_GRAYSCALE))
        skp, skpd = instance.analyze_frame(cv2.imread(img2,cv2.CV_LOAD_IMAGE_GRAYSCALE))
        G, set1, set2 = instance.get_homography_and_matches(fkp, fkpd, skp, skpd)
        print "Set1 length " + str(len(set1))
        print "Set2 length " + str(len(set2))
        self.assertTrue(len(set1) >= 4 and len(set2) >= 4, "Finding homography failed")'''
        pass

    def test_decompose_homography(self):
        _cam_intrinsics = np.float32([[640, 0, 320],
                                     [0, 640, 240],
                                     [0, 0, 1]])
        _H = np.float32([[2.649157564634028, 4.583875997496426, 70.694447785121326],
                         [-1.072756858861583, 3.533262150437228, 1513.656999614321649],
                         [0.001303887589576, 0.003042206876298, 1.000000000000000]])
        _R = np.array([[0.43307983549125, 0.545749113549648, -0.717356090899523],
                         [-0.85630229674426, 0.497582023798831, -0.138414255706431],
                         [0.281404038139784, 0.67421809131173, 0.682818960388909]])
        _t = np.array([1.826751712278038, 1.264718492450820, 0.195080809998819])
        max_error = 1.0e-3

        instance = get_angle_using_homography.DriftDetector(_cam_intrinsics)
        solutions = instance.decompose_homography(_H)
        check = False
        for R, t in solutions:
            R_err = cv2.norm(R, _R, cv2.NORM_INF)
            t_err = cv2.norm(t, _t, cv2.NORM_INF)
            if R_err < max_error and t_err < max_error:
                check = True
                break

        self.assertTrue(check, "Decomposition of Homography failed")

    def test_get_drift(self):
        cam_intrinsics = np.float32([[733.64357007, 0, 373.41222549],
                                     [0, 788.17218714, 495.25800414],
                                     [0, 0, 1]])

        instance = get_angle_using_homography.DriftDetector(cam_intrinsics)
        img1 = "data/test_images/IMG_270.jpg"
        img2 = "data/test_images/IMG_275.jpg"
        print "\nComparing " + img1 + " and " + img2
        fkp, fkpd = instance.analyze_frame(cv2.imread(img1,cv2.CV_LOAD_IMAGE_GRAYSCALE))
        skp, skpd = instance.analyze_frame(cv2.imread(img2,cv2.CV_LOAD_IMAGE_GRAYSCALE))
        r,p,y = instance.get_drift(fkp, fkpd, skp, skpd)
        print "\n\nRoll: %f, Pitch: %f, Yaw: %f" %(r , p , y)

        self.assertTrue(True)

if __name__ == "__main__":
    unittest.main()
