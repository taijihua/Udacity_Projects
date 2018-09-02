## Advanced Lane Finding
### This is to fullfil the requirement for Udacity Self-Driving Car Engineer Nanodegree project 2 assignment

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/camera_calibration.png "Undistorted_chessboard"
[image2]: ./output_images/camera_calibration_test_image.png "Undistorted_test_image"
[image3]: ./output_images/binary-1.png "Binary Example"
[image4]: ./output_images/binary-2.png "Binary Example"
[image5]: ./output_images/binary-3.png "Binary Example"
[image6]: ./output_images/binary-4.png "Binary Example"
[image7]: ./output_images/binary-5.png "Binary Example"
[image8]: ./output_images/binary-6.png "Binary Example"
[image9]: ./output_images/binary-7.png "Binary Example"
[image10]: ./output_images/Perspective-1.png "Perspective transform Example"
[image11]: ./output_images/Perspective-2.png "Perspective transform Example"
[image12]: ./output_images/Perspective-3.png "Perspective transform Example"
[image13]: ./output_images/Lane_Fitting-2.png "Lane fitting Example"
[image14]: ./output_images/Sample_output.png "Final output"

[video1]: ./output_videos/project_video_output.mp4 "Video"

---

### My response to the [rubric points](https://review.udacity.com/#!/rubrics/571/view) (by Udacity)

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the IPython notebook located in "P2.ipynb".  

The camera calibration is done by specifying the (x, y, z) coordinates of the chessboard corners (`objpoints` in the code) in the world (the chessboard is assumed to be fixed on the (x, y) plane at z=0, and the object points are the same for each calibration image), and then identifying corresponding `imgpoints` on each calibration image by using `cv2.findChessboardCorners()` function.  

I then used the specified `objpoints` and identified `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to images using the `cv2.undistort()` function and obtained the following example result: 

![alt text][image1]


### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

As mentioned above, the `cv2.undistort()` function is applied to test images and the following is an example:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

I used a combination of color and gradient thresholds to generate a binary image (thresholding steps in "P2.ipynb" cell 3, as well as cell 25 which is encapsulated into a class implementation for whole pipeline).  Here's an example of my output for this step, as well as some intermediate analysis images (e.g, HLS channel images, differnt masks etc).

The original RGB image (rectified using method above), along with its grayscale image:
![alt text][image3]

Convert into HSL format:
![alt text][image4]

Tried two thresholding for extracting the yellow lines and white lines:
![alt text][image5]

Gradient on grayscale and individual HLS channels:
![alt text][image6]
![alt text][image7]

Applied a region mask (trapezoid as indicated by the yellow lines below) to only focus on lane area in front of the camera
![alt text][image8]

combined differnt thresholding/mask to obtain the final binary image:
![alt text][image9]

The final binary image was obtained by using combined color thresholding (white and yellow), gradient on S channel, and region mask, after numerous trials and errors.

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform is in 4th cell of "P2.ipynb" (as well as in 25th cell in the class implementation).  To calculate the perspective transform matrix 'M' (original -> top down) and 'Minv' (top down -> original), I manually picked 4 points in original image (undistorted) that should approximately form a rectangle area in real world (trepozoid in original image), and their corresponding desired positions in the topdown view:
pts = np.array([[323, 650], [1008, 650], [730, 475], [560, 475]])
pts_topdown = np.array([[323, 650], [1008, 650], [1008, 300], [323, 300]])
The 'M' and 'Minv' can then be calculated by using `cv2.getPerspectiveTransform()` function. And `cv2.warpPerspective` function can be used for transforming images between diffenrt views by supplying the 'M' and 'Minv'.

I verified that my perspective transform was working as expected, as in images below

![alt text][image10]
![alt text][image11]

Here is the warped top-down view image based on the binary image in previous step.
![alt text][image12]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

In "P2.ipynb', the 5th cell (25th cell in class definition) implemented lane-pixels identification by using sliding window approach. The 25th cell also implemented a lane-pixels identification method (function `findLaneLinesWithPreviousPolyfit()`) by using previous frame's fitted curve. So in the actual video processing, sliding window approach is used on first frame or whenever the fitting failed in last frame, and other frames used previous fitted curve for searching the lane pixels.

After lane pixels are found, a 2nd order polynomial fitting is applied to left and right lane pixels 

Here is an example of the sliding window search and the resulted poly fitting (red line).
![alt text][image13]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

In "P2.ipynb', the 6th cell (25th cell with class implementation) implemented radius of curvature calculation and vehicle position with respect to center.
To calculate the radius of curvature, the approach was to convert the fitted polynomial equation into real world metrics (meters) to obtain the 'new' fitting coefficients, and then calculate the curvature in meters by applying the method as taught in Udacity class (also tutorial in http://www.intmath.com/applications-differentiation/8-radius-curvature.php)

To calculate the vehicle position with respect to center, the center point of the image (assumed to be the position of the center of vehicle) is compared with the center point of the two fitted lane lines.



#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

Here is an example image coming out of the pipeline with lane area clearly identified.

![alt text][image14]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to project video result](./output_videos/project_video_output.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

In addition to the steps above, in my code 'P2.ipynb' i also implemented additional processing to help smooth out the result and reject obvious wrong lane fittings, they are:
* sanity check on the fitted lanes (in cell 25 function `sanityCheckLaneLines()`)
    If fitted coefficients are empty (which usually means no lane pixels found), use previous frame's value
    If the distance between the two fitted lane lines are smaller than 2 meter or greater than 6 meter, reject the current fitting results since normal lane width is around 3.7 meter.
* Temporal moving average on fitted lane lines (in cell 25 function `sanityCheckLaneLines()`)
    use weighted moving avarage of the fitted lane lines as the output for current frame's fitted values. In the code i used 3 frames, with the weight assigned to be 0.5, 0.3 and 0.2 for current frame, the last frame and the frame before the last.
    
Overall, the pipeline works well on the project video, and reasonably ok on the 'challenge project video', but quite off on the 'harder challenge project video'

Here's a [link to challenge project video result](./output_videos/challenge_video_output.mp4)

Here's a [link to harder challenge project video result](./output_videos/harder_challenge_video_output.mp4)

There are some failure in 'challenge project video' due to very strong edge (shadow) near the left lane being picked up as 'lane pixels' and thus affected the polynomial fitting. The thresholding in color space and gradient may need further tweak to filter out these pixels, or more strict sanity check on the lane pixels or fitted values.

There are quite some failure in 'harder challenge project video', possibly due to: some sharpe turns that made lane line outside my region mask (trepozoid) or even outside camera view, some strong sunlight causing over exposure to camera and some shadow adding lots noise to the thresholded binary images, some motorcyclist moving in and out of lane could also be falsely picked up as lane pixels. Potential improvement can include: adjust the region mask based on lanes found in previous frames instead of the fixed region mask in current code; some more strict lane pixel sanity check to help remove strong sunlight and shadow, maybe use history frames' information to infer if the lane pixels found are valid or not.

