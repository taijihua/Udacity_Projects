# **Traffic Sign Recognition** 

## Writeup


---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./writeup_images/ClassHistogram.png "Visualization"
[image2]: ./writeup_images/randomSamples.png "Visualization"
[image3]: ./writeup_images/grayscale.png "grayscale"
[image4]: ./test_images/test_image_1.jpg "Traffic Sign 1"
[image5]: ./test_images/test_image_2.jpg "Traffic Sign 2"
[image6]: ./test_images/test_image_3.jpg "Traffic Sign 3"
[image7]: ./test_images/test_image_4.jpg "Traffic Sign 4"
[image8]: ./test_images/test_image_5.jpg "Traffic Sign 5"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/taijihua/Udacity_Projects/tree/master/Project3_Traffic_Sign_Classifier/Traffic_Sign_Classifier.ipynb)

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the numpy library to calculate summary statistics of the traffic
signs data set:

* Number of training samples = 34799
* Number of validation samples = 4410
* Number of testing samples = 12630
* Image data shape = (32, 32, 3)
* Number of unique classes = 43

#### 2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. It is a histogram chart showing the data distrubition among classes

![data histogram][image1]

Here are some sample images:

![random sample images][image2]

### Design and Test a Model Architecture

#### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

As a first step, I decided to use grayscale image because this can reduce the dimension of feature space, and thus reduce computation time and help overcome overfitting, and yet grayscale images preserve enough information to be recognized (human eye can recognize grayscale traffic signs). Note that my conversion of RGB to grayscale is configurable in function LeNet_modified(x, n_channels, keep_prob) by specifying n_channels=1 (use original RGB if n_channels is set to 3)

Here is the example traffic sign images (same samples as above) in grayscale.

![my grayscale image][image3]

As a last step, I normalized the image data by subtracting each pixel value by 128 and then dividing by 128, this is to center the input data around zero mean and range -1 to 1, in order to help numerical optimization in gradient descent.

I didn't generate additional data since the accuracy was good enough to meet the requirement. I could use data augmentation such as slight rotation, mirroring, or adding slight noise to the images.


#### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model is modified from LeNet structure, consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x1 grayscale image   			     	| 
| Convolution 5x5     	| 1x1 stride, VALID padding, outputs 28x28x6 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 14x14x6 				    |
| Convolution 5x5	    | 1x1 stride, VALID padding, outputs 10x10x16	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 5x5x16 				    |
| Fully connected		| output 120   									|
| Dropout       	    | set to 0.5 at training, 1 at test                 	|
| RELU					|												|
| Fully connected		| output 84   									|
| Dropout       	    | set to 0.5 at training, 1 at test                 	|
| RELU					|												|
| Fully connected		| output 43   									|
| Softmax				|            									|

 


#### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

The following parameters were used during the training:
* EPOCHs = 30
* Batch size = 32
* Learning rate = 0.001
* Optimizer is tf.train.AdamOptimizer

#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set accuracy = 0.997
* validation set accuracy = 0.950
* test set accuracy = 0.946

My model was modified from LeNet, which is a relatively shallow CNN model that primarily includes 2 convolution layers and 3 fully connected layers. Traffic signs involve intrinsically spatial features, which justifies the choice of using convolution layers. Given that the dimension of the input image is not big (32 X 32), this shallow architecture should be enough to achieve high accuracy. During the initial training, i found high accuracy on trainning set while relative low on validation set, which indicates overfitting. I then chose to add two drop out layers to force more neurons to learn the features and these drop out layers helped.


 

### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![alt text][image4]
![alt text][image5]
![alt text][image6]
![alt text][image7]
![alt text][image8]


#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| Double Curve      	| No entry    									| 
| Speed limit (30km/h) | End of all speed and passing limits 										|
| Speed limit (70km/h) | Speed limit (70km/h)											|
| Speed limit (60km/h)	| Priority road					 				|
| Slippery Road			| Slippery road      							|


The model was only able to correctly guess 2 of the 5 traffic signs, which gives an accuracy of 40%. This indicates overfitting of the training data and lack of generalization of the model...

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 11th cell of the Ipython notebook.

For the first image, the model is relatively sure that this is a stop sign (probability of 0.6), and the image does contain a stop sign. The top five soft max probabilities were

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .60         			| Stop sign   									| 
| .20     				| U-turn 										|
| .05					| Yield											|
| .04	      			| Bumpy Road					 				|
| .01				    | Slippery Road      							|


For the second image ... 

### (Optional) Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)
#### 1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?



