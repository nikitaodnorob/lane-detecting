# Lane Detecting
This program detects the lane on the images and video files. The main idea is using of small horizontal lines named by sensors

# Algorithm
The source image (or video) may be taken from a DVR. For example:

![source image](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/1.png)

## 1. The work area
An all image is not interesting for us. We need to take the part of image which contains only road and excludes sky and a car bonnet. It will be a trapezoidal area of the image:

![work area](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/2.png)

Properties of this trapezoid, such as the horizon point position, are contained in the `CameraProperties` static class

## 2. Convert to HSV
The RGB color model is not comfortable for lane detection. So, we convert the source image to HSV model:

![hsv](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/3.png)

## 3. Find contours
Now we have to find contours on this image. The Canny detector is used for that in the project. Then on result binary image we find contours via OpenCV and filter small of them:

![contours](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/5.png)

## 4. Create sensors
The sensor is a small horizontal line placed on the image. This line is the work area of the sensor. Each sensor try to find the lane in its work area. We put sensors onto the image and delete those which do not cover contours:

![contours which are covering sensors](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/6.png)

## 5. Filter sensors
* Then we filter sensors by middle color in the work area, searching the light colors. 
* Besides, the lane color differs from the road color, so we filter sensors also by difference between middle color inside sensor and middle color outside sensor
* The lane is the long lines in the image. So, there are many sensors covering the one lane line in the image. We filter sensors by existing other sensors near current one

![filtered sensors](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/7.png)

## 6. Group sensors
Now we have many sensors placed on all image. We calculate for each sensor an angle from it to the horizon point. Then we group sensors by angles and distances for other sensors:

![grouped sensors](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/9.png)

## 7. Draw lines
In the end we analyze the received groups by count of sensors in them and dispercy of angle. The most suitable groups we name to lane line and then draw line though these sensors:

![result](https://github.com/nikitaodnorob/lane_detecting/blob/master/images/10.png)

## Remark
This program well detects a bright lane in the morning and in the night, but has some problems with old and dirty lane. The algorithm is need to improving by making better sensors filtering and groupping
