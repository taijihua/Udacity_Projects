# **Behavioral Cloning** 

## My Write up

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/center_2018_10_22_20_52_04_897.jpg "center image"
[image2]: ./examples/center_2018_11_19_23_16_18_825.jpg "side image from right"
[image3]: ./examples/center_2018_11_19_23_19_31_998.jpg "side image from left"
[image4]: ./examples/Histogram_original_data.png "histogram of original data"
[image5]: ./examples/history_loss.png "training history"


## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode (slight modification from Udacity's original file)
* model.h5 containing a trained convolution neural network 
* writeup_report.md summarizing the results (you are reading it)

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.
The drive.py is modified slightly from Udacity's provided file to convert image from RGB to BGR, in order to match the training samples that are loaded by OpenCV (BGR format)

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model consists of a convolution neural network with the following layers from input to output:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 160x320x3 image   			     	        | 
| Cropping2D       		| output 90x320x3 (top 50 and bottom 20 rows removed   	        | 
| Lambda         		| convert pixels from 0-255 to -0.5-0.5   			     	        | 
| Convolution 5x5     	| default 1x1 stride, VALID padding, outputs 86x316x24 	|
| RELU					|												|
| BatchNorm				|												|
| Max pooling	      	| 2x2 stride,  outputs 43x158x24 				    |
| Dropout       	    | 0.5 for training                 	|
| Convolution 5x5	    | 1x1 stride, VALID padding, outputs 39x154x36	|
| RELU					|												|
| BatchNorm				|												|
| Max pooling	      	| 2x2 stride,  outputs 19x77x36 				    |
| Dropout       	    | 0.5 for training                 	|
| Convolution 5x5	    | 1x1 stride, VALID padding, outputs 15x73x48	|
| RELU					|												|
| BatchNorm				|												|
| Max pooling	      	| 2x2 stride,  outputs 7x36x48 				    |
| Dropout       	    | 0.5 for training                 	|
| Convolution 3x3	    | 1x1 stride, VALID padding, outputs 5x34x64	|
| RELU					|												|
| BatchNorm				|												|
| Dropout       	    | 0.5 for training                 	|
| Fully connected		| output 100   									|
| RELU					|												|
| BatchNorm				|												|
| Dropout       	    | 0.5 for training                 	|
| Fully connected		| output 50   									|
| RELU					|												|
| BatchNorm				|												|
| Dropout       	    | 0.5 for training                 	|
| Fully connected		| output 10   									|
| RELU					|												|
| BatchNorm				|												|
| Dropout       	    | 0.5 for training                 	|
| Fully connected		| output 1   									|



In this model, the input data is preprocessed using keras cropping layer to discard irrelavant portion of the image , and normalized to (-0.5, 0.5) range using a Keras lambda layer. There are multiple convolutional layers with kernel size 5x5 or 3x3 and RELU activation. The intermediate activations are also normalized by using batch norm to improve training convergence and thus performance.Dropout layers are added as well to help combat overfitting.

#### 2. Attempts to reduce overfitting in the model

The model contains dropout layers in order to reduce overfitting, and batch normalization to help training convergence.

The model was trained and validated on different data sets to ensure that the model was not overfitting. The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The following parameters were used for training the model:
* EPOCHs = 70 (EarlyStopping call back is used)
* Batch size = 32
* Learning rate = 0.001
* Optimizer is 'adam'

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road. Both track 1 and track 2 from the simulator were used for collecting data. and data augmentation was applied on training samples by horizontally flipping images, and using left and right camera images with steering offset.

For details about how I created the training data, see the next section. 

### Architecture and Training Documentation

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to find a model that can achieve low training loss, low validation loss, and ultimately good generalization capability (can do autonomous driving on track in the simulator)

I started with a convolution neural network model similar to the NVidia Network introduced by Udacity class. The model is relatively simple (not too deep, thus less prone to overfitting) and requires not too much computation. I made a small change to remove the 2x2 stride in convolution layers, under the thought that i could use 2x2 maxpooling layers instead which may better reserve important features from the data. After first few trainings by tweaking learning rate and batch size, i was not quite satisfied with the validation loss, which was quite high compared to training loss. There seems to have some overfitting problem, or my hyperparameter selection might not be appropriate. After some literature reading and online search, i learned that batch normalization could help with training convergence and has become popular recently. I therefore added batch normalization layers into the model. and Dropout layers are also added to help combat overfitting.


After tuning the model to achieve low training loss and similar validation loss, the next step was to run the simulator to see how well the car was driving around track one. I did found a few spots where the vehicle fell off the track, for example in my case the car didn't perform well on the curve right before on the bridge, and a few curves that only have one side marking in track 1. To improve the model prediction in these cases, I collected some addtional data around those curves, especially along the road edges, in order to sure the car could learn how to stay away from the side of road.

One more important finding (thanks to Udacity reviewer for pointing it out from my previous submission, i should have found this out earlier...) is that in the original drive.py the images are loaded in RGB while the training was done using opencv loading images (BGR), I then modified drive.py to switch channels to be consistent with training process.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture are summarized in the above section (Model Architecture and Training Strategy)

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded a few laps on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image1]

I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to steer towards center of road in these situations.

![alt text][image2]
![alt text][image3]


Then I repeated this process on track two in order to get more data points.
The data were then randomly divided into training samples (80% of data) and validation samples (20% of data). 
Overall I have the following number of samples:
* Number of training samples: 19884
* Number of validation samples: 4972

Here is a histogram of the steering angles among all data collected:
![alt text][image4]

During the training process, data augmentation was applied with the following:
* images were flipped left-right with the output label (steering angle) also flipped direction (+ or -)
* left and right camera images were used by adding offset of 0.2 to steering angle.

The above data augmentation was only applied for training samples (thus training samples become 19884*4 = 79536), but NO data augmentation was applied on the validation samples since i would like to test out the performance on 'real' samples.

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The number of epochs was selected based on the decay of loss values. I used an adam optimizer which can adaptively update individual parameter's learning rate. EarlyStopping call back is used to terminate training earlier than max epochs if the losses become stable. Snapshot of each epoch was saved by using ModelCheckpoint call back.
Plot of training history on train loss and validation loss values
![alt text][image5]
