import cv2
import csv
import matplotlib.pyplot as plt
get_ipython().magic('matplotlib inline')
import numpy as np
import sklearn
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

# My local data folder for images
dataFolder = '/Project4_Behavior_Cloning/All_training_data/'

lines = []
# My first bunch collection of data
with open(dataFolder+'driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:  
        lines.append(line)        

# My second bunch collection of data, mainly for some challenging curves
with open(dataFolder+'driving_log_additional_track1.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:  
        lines.append(line)
# My third round data collection on track 2
with open(dataFolder+'driving_log_additional_track2.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:  
        lines.append(line)


train_samples, validation_samples = sklearn.model_selection.train_test_split(lines, test_size=0.2)
# show overview number of samples
print("=====samples count (before data augmentation)=======")
print("Number of training samples: {}".format(len(train_samples)))
print("Number of validation samples: {}".format(len(validation_samples)))

# define generator function, use 'use_augmentation' flag to toggle on/off data augmentations (flip, left/right cameras)
def generator(samples, batch_size=32, use_augmentation = True):
    flagList = [0]*len(samples)
    if use_augmentation:
        #load an image and process it based on its corresponding value in 'flagList': 
        #       if 0, use original center image; 
        #       if 1, use flipped center image  
        #       if 2, use left camera image, measurement + 0.2
        #       if 3, use right camera image, measurement - 0.2
        flagList.extend([1]*len(samples))        
        flagList.extend([2]*len(samples))
        flagList.extend([3]*len(samples))
        samples = samples*4
    
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates        
        samples, flagList= sklearn.utils.shuffle(samples, flagList)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            batch_flags = flagList[offset:offset+batch_size]

            images = []
            angles = []
            for ix, batch_sample in enumerate(batch_samples):
                if batch_flags[ix]==0 or batch_flags[ix]==1:  #use center image, or flipped center image
                    name = dataFolder+'/IMG/'+batch_sample[0].split('/')[-1]
                elif batch_flags[ix]==2:  # use left image
                    name = dataFolder+'/IMG/'+batch_sample[1].split('/')[-1]
                elif batch_flags[ix]==3:  # use right image
                    name = dataFolder+'/IMG/'+batch_sample[2].split('/')[-1]                
                current_image = cv2.imread(name)
                current_angle = float(batch_sample[3])
                if batch_flags[ix]==1:
                    current_image = np.fliplr(current_image)
                    current_angle = -current_angle
                elif batch_flags[ix]==2:
                    current_angle = current_angle + 0.2
                elif batch_flags[ix]==3:
                    current_angle = current_angle - 0.2
#                 print(name)
#                 print(current_image.shape)
                images.append(current_image)
                angles.append(current_angle)

            X_train = np.array(images)
            y_train = np.array(angles)
#             print(batch_samples)
#             print(X_train.shape)
#             print(y_train.shape)
            yield sklearn.utils.shuffle(X_train, y_train)

# build up the model
from keras.models import Sequential, Model
from keras.layers import Flatten, Dense, Lambda, BatchNormalization, Dropout
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Cropping2D
def CreateNvidiaNetwork():
    dropRatio1 = 0.5 # discard dropRatio(0.2)
    model = Sequential()
    model.add(Cropping2D(cropping=((50, 20), (0, 0)),  input_shape=(160, 320, 3))) #input become 90X320X3
    model.add(Lambda(lambda x: x / 255.0 - 0.5))
    #model.add(Convolution2D(24, (5, 5), subsample=(2,2), activation="relu")) #feature become 43X158X24
    model.add(Convolution2D(24, (5, 5),  activation="relu"))
    model.add(BatchNormalization())
    model.add(MaxPooling2D(pool_size = (2, 2)))
    model.add(Dropout(dropRatio1))
    #model.add(Convolution2D(36, (5, 5), subsample=(2,2), activation="relu")) #feature become 20X77X36
    model.add(Convolution2D(36, (5, 5),  activation="relu"))
    model.add(BatchNormalization())
    model.add(MaxPooling2D(pool_size = (2, 2)))
    model.add(Dropout(dropRatio1))
    #model.add(Convolution2D(48, (5, 5), subsample=(2,2), activation="relu")) #feature become 8X37X48
    model.add(Convolution2D(48, (5, 5),  activation="relu"))
    model.add(BatchNormalization())
    model.add(MaxPooling2D(pool_size = (2, 2)))
    model.add(Dropout(dropRatio1))
    model.add(Convolution2D(64, (3, 3), activation="relu")) #feature become 6X35X64
    model.add(BatchNormalization())
    model.add(Dropout(dropRatio1))
    #model.add(Convolution2D(64, (3, 3), activation="relu")) #feature become 4X33X64
    #model.add(BatchNormalization())
    #model.add(MaxPooling2D(pool_size = (2, 2)))
    #model.add(Dropout(dropRatio1))
    model.add(Flatten())
    model.add(Dense(100, activation="relu"))
    model.add(BatchNormalization())
    model.add(Dropout(dropRatio1))
    model.add(Dense(50, activation="relu"))
    model.add(BatchNormalization())
    model.add(Dropout(dropRatio1))
    model.add(Dense(10, activation="relu"))
    model.add(BatchNormalization())
    model.add(Dense(1))    
    return model

# compile and train the model
from keras import optimizers
from keras.callbacks import EarlyStopping, ModelCheckpoint
model = CreateNvidiaNetwork()    
model.compile(loss='mse', optimizer='adam')
myBatchSize = 32
train_generator = generator(train_samples, myBatchSize, use_augmentation = True)
validation_generator = generator(validation_samples, myBatchSize, use_augmentation = False)
trainHistory = model.fit_generator(train_generator, steps_per_epoch= int(len(train_samples)*4/myBatchSize), \
                                    validation_data=validation_generator, validation_steps=int(len(validation_samples)/myBatchSize), \
                                   epochs=70, verbose = 1,
                                  callbacks=[EarlyStopping(monitor='val_loss', min_delta=0.0001, patience=3),
                                            ModelCheckpoint("model2-epoch-{epoch:02d}-valloss-{val_loss:.2f}.h5", monitor='val_loss', verbose=0, save_best_only=False)])
import pickle
with open("model-log1.pkl", "wb") as f:
    pickle.dump(trainHistory.history, f)


### load the saved history log
with open("model-log1.pkl", "rb") as f:
    history = pickle.load(f)
print(history.keys())

### plot the training and validation loss for each epoch
plt.plot(history['loss'])
plt.plot(history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()
