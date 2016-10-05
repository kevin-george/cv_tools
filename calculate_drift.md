<div id="table-of-contents">
<h2>Table of Contents</h2>
<div id="text-table-of-contents">
<ul>
<li><a href="#orgheadline37">1. True Magnetic North</a>
<ul>
<li><a href="#orgheadline1">1.1. Statement: The image is taken at the magnetic north and we take another image at the same magnetic north. We find the translation and orientation shift vector to estimate the drift. The analysis needs to be done in real-time.</a></li>
<li><a href="#orgheadline2">1.2. Physical setup: The camera is part of an Android phone and is rotating on its own axis</a></li>
<li><a href="#orgheadline5">1.3. Unknowns:</a>
<ul>
<li><a href="#orgheadline3">1.3.1. RPM of movement</a></li>
<li><a href="#orgheadline4">1.3.2. Orientation of camera</a></li>
</ul>
</li>
<li><a href="#orgheadline8">1.4. Knowns:</a>
<ul>
<li><a href="#orgheadline6">1.4.1. Speed of rotation shall be constant(There exists an encoder on the motor)</a></li>
<li><a href="#orgheadline7">1.4.2. The camera origin shall remain at the same position</a></li>
</ul>
</li>
<li><a href="#orgheadline36">1.5. Algorithm</a>
<ul>
<li><a href="#orgheadline14">1.5.1. Calibrate camera</a></li>
<li><a href="#orgheadline17">1.5.2. Estimate drift by computing rotation and translation between the two camera frames</a></li>
<li><a href="#orgheadline35">1.5.3. Tests</a></li>
</ul>
</li>
</ul>
</li>
</ul>
</div>
</div>

# True Magnetic North<a id="orgheadline37"></a>

## Statement: The image is taken at the magnetic north and we take another image at the same magnetic north. We find the translation and orientation shift vector to estimate the drift. The analysis needs to be done in real-time.<a id="orgheadline1"></a>

init(magnetic<sub>north</sub><sub>image</sub>)
drift = get<sub>drift</sub>(estimated<sub>magnetic</sub><sub>north</sub><sub>image</sub>)

## Physical setup: The camera is part of an Android phone and is rotating on its own axis<a id="orgheadline2"></a>

## Unknowns:<a id="orgheadline5"></a>

### RPM of movement<a id="orgheadline3"></a>

### Orientation of camera<a id="orgheadline4"></a>

## Knowns:<a id="orgheadline8"></a>

### Speed of rotation shall be constant(There exists an encoder on the motor)<a id="orgheadline6"></a>

### The camera origin shall remain at the same position<a id="orgheadline7"></a>

## Algorithm<a id="orgheadline36"></a>

### Calibrate camera<a id="orgheadline14"></a>

1.  Resize the image to 1000x1000 if the the calibration routine is taking too long

2.  The background to the chessboard needs to be white so corners can be detected easily

3.  The chessboard needs to be asymmetric i.e. number of rows should not be equal to number of columns

4.  The cornerSubPix function may make things worse and cause drawChessboardSquares to stop functioning

5.  This gives us the camera intrinsics matrix K

### Estimate drift by computing rotation and translation between the two camera frames<a id="orgheadline17"></a>

1.  Convert image to grayscale
2.  Use a feature detector method e.g. SIFT/SURF/FAST to detect the keypoints
3.  Generate feature descriptor vectors for the keypoints
4.  Match pairs of keypoints between images(the two images from estimated magnetic north) using FLANN(Nearest Neighbor). Perform Lowe's ratio test for outlier detection.

1.  Compute Rotation and translation form Fundamental Matrix

    1.  Compute the fundamental matrix using RANSAC/LMEDS to further remove outliers
    2.  F = K' E K
        so, E = K'<sup>-1</sup> F K<sup>-1</sup> 
        K' is the transpose, K<sup>-1</sup> is the inverse
    3.  Given the essential matrix(E), decompose using SVD which gives us 4 possible Rotational and Translation vectors(R & t)
    4.  Find correct R & t using positive depth constraint
    5.  Using the rotational vector compute roll, pitch and yaw. These are Euler angles which are computed from the rotational matrix using the relationship between the 3 axes orthogonal to x,y,z <http://planning.cs.uiuc.edu/node103.html>

2.  Compute Rotation and translation from Homography

    1.  Compute the homography using RANSAC/LMEDS to remove outliers that don't fit the homography
    2.  Decompose the homography using <https://hal.archives-ouvertes.fr/file/index/docid/174739/filename/RR-6303.pdf> to get 4 possible R & t
    3.  Find correct R & t using positive depth constraint
    4.  Compute roll, pitch and yaw using R

### Tests<a id="orgheadline35"></a>

1.  WAITING Test cases for calibration module

    1.  The reprojection error needs to be below 0.5
    
    2.  K[0,0] and K[1,1] needs to be as close as possible as they are the focal length along the x and y axis
    
    3.  Print chessboard on matte/stock paper and stick onto a flat wood surface(planar?). Better printing can be done at Office Depot.
    
    4.  Check if undistortion improves results after calculating K

2.  DONE SIFT/SURF/ORB Which one works better and faster?

    1.  SIFT and SURF generate the most number of features with stationary images. ORB is the fastest and generates less but just as accurate features

3.  IN-PROGRESS Test cases for computation of Homography

    1.  Which method is best for computing the homography - RANSAC or LMEDS? What paramenters work best?
    
    2.  Calculate re-projection error after generation of Homography

4.  IN-PROGRESS Test cases for decomposition of Homography

    1.  Used the test case from openCV
    
    2.  Possible theoretical test -> H ~= R + t.n<sup>T</sup>

5.  TODO Test cases for finding correct solution from decomposition of Homography

6.  IN-PROGRESS Test cases for images set with known rotation

    1.  5 images created with 5 degrees yaw between consecutive images for test set
    
    2.  Results:
    
        <table border="2" cellspacing="0" cellpadding="6" rules="groups" frame="hsides">
        
        
        <colgroup>
        <col  class="org-left" />
        
        <col  class="org-left" />
        
        <col  class="org-right" />
        
        <col  class="org-right" />
        </colgroup>
        <tbody>
        <tr>
        <td class="org-left">Image 1</td>
        <td class="org-left">Image 2</td>
        <td class="org-right">true yaw</td>
        <td class="org-right">calc yaw</td>
        </tr>
        
        
        <tr>
        <td class="org-left">-------</td>
        <td class="org-left">-------</td>
        <td class="org-right">--------</td>
        <td class="org-right">--------</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-left">IMG<sub>275</sub></td>
        <td class="org-right">5</td>
        <td class="org-right">3.47</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>275</sub></td>
        <td class="org-left">IMG<sub>280</sub></td>
        <td class="org-right">5</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-left">IMG<sub>280</sub></td>
        <td class="org-right">10</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>280</sub></td>
        <td class="org-left">IMG<sub>290</sub></td>
        <td class="org-right">10</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-left">IMG<sub>275</sub></td>
        <td class="org-right">15</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>275</sub></td>
        <td class="org-left">IMG<sub>290</sub></td>
        <td class="org-right">15</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-left">IMG<sub>290</sub></td>
        <td class="org-right">20</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>275</sub></td>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-right">-5</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>290</sub></td>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-right">-20</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        
        
        <tr>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-left">IMG<sub>270</sub></td>
        <td class="org-right">0</td>
        <td class="org-right">&#xa0;</td>
        </tr>
        </tbody>
        </table>
