# **Finding Lane Lines on the Road** 

## This is to fullfil the requirement for Udacity Self-Driving Car Engineer Nanodegree project 1 assignment

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)
[image_original]: ./temp_output/original.png
[image1]: ./temp_output/intermediate-1.png
[image2]: ./temp_output/intermediate-2.png
[image3]: ./temp_output/intermediate-3.png
[image4]: ./temp_output/intermediate-4.png
[image5]: ./temp_output/intermediate-5.png

---

### Reflection

### 1. My pipeline and how I modified the draw_lines() function.

My pipeline consisted of 6 steps, use the following image as an example:
![alt text][image_original]
##### 1. convert color image to grayscale:  
![alt text][image1]
##### 2. apply Gaussian blurring to the gray scale (3X3, this is optional since Canny Edge implicitly does Gaussian blur with 5X5)
##### 3. Canny edge detection (threshold 50 and 200)
![alt text][image2]
##### 4. apply region mask (area between the following points)
            left_bottom = [int(image.shape[1]*0.05), image.shape[0]]
            right_bottom = [int(image.shape[1]*0.95), image.shape[0]]
            left_top = [int(image.shape[1]*0.48), int(image.shape[0]*0.58)]
            right_top = [int(image.shape[1]*0.52), int(image.shape[0]*0.58)]
![alt text][image3]
##### 5. Hough transform to get staight lines
![alt text][image4]
##### 6. Overlay the lines on original color image
![alt text][image5]


In order to draw a single line on the left and right lanes, I modified the draw_lines() function by doing the following:
*   calculate the slopes of all fitted lines by Hough transform
*   Seperate the lines to two buckets of left lane or right lane based on slope (>0.2 or <-0.2, i chose a cutoff value 0.2 rather than 0 and it seemed to help remove some noisy horizontal lines)
*   for each bucket, fit a single lane line by combining all end points
*   overlay the single left lane line and single right lane line

---
### 2. Potential shortcomings and possible improvements


There are couple of potential shortcomings with current implementation:
1. <b>Shadows</b> seem to have some impact on the edge detection, i have implemented an enhanced lane line detection by using color thresholding (cv2.inRange function) in HSV space at the end of the notebook, which would be more robust to shadows. 
2. The enhanced lane line detection using color thresholding is tuned for yellow or white lanes only. There could be <b>other colors for lane markings</b> on the road which need to be further added to the color thresholding
2. <b>Objects within the region mask</b> could impact the lane line fitting as well. For example, a car in the same lane right in front of the camera would be picked up by the edge detection or color thresholding (if the car is white or yellow), also some objects on the ground may also be picked up, especially if they have straight line edge. 
Potential solution could be to look at the distributions of all fitted slopes and intercepts, and remove those obvious outliars (e.g, outside 3 standard deviations) before applying averaging/fitting to get single left or right lane.
3. Another shortcomings of current lane line detection is that <b>it only fits straight lines</b>, but if a vehicle is going through a curve with lane markings not on straight line. A potential solution could be to fit a 2nd or higher order polynomial curve to lane lines in those cases, instead of straight line. 
4. Sometimes the <b>lane line markings on left or right side could be temparary missing</b> (or not good enough to fit a single line) at certain time the car is driving. one potential solution is to add temporal memory of lane line tragectory, in this case (lane line temporarily missing) it could follow previously detected lane line. Also by applying temporal filter, we could smooth out the lane line tragectory with time and reduce temporal lane line zig-zag's.


