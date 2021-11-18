import os
import pathlib
from self_driving_car_pkg.config import config
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.offsetbox import AnchoredOffsetbox, TextArea, HPacker
from sklearn.model_selection import train_test_split

from tensorflow.keras.preprocessing.image import  img_to_array, load_img
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.layers import Conv2D, MaxPool2D, Dense, Flatten, Dropout
from tensorflow.keras.models import Sequential,load_model # import load_model function to load trained CNN model for Sign classification
import tensorflow as tf
print(tf.__version__)#2.4.1 Required

Evaluate_Model = True


NUM_CATEGORIES = 0
sign_classes = ["speed_sign_30","speed_sign_60","speed_sign_90","stop","left_turn","No_Sign"] 

def load_data(data_dir):
    '''
    Loading data from Train folder.
    
    Returns a tuple `(images, labels)` , where `images` is a list of all the images in the train directory,
    where each image is formatted as a numpy ndarray with dimensions IMG_WIDTH x IMG_HEIGHT x 3. 
    `labels` is a list of integer labels, representing the categories for each of the
    corresponding `images`.
    '''
    global NUM_CATEGORIES
    images = list()
    labels = list()
    for category in range(NUM_CATEGORIES):
        categories = os.path.join(data_dir, str(category))
        for img in os.listdir(categories):
            img = load_img(os.path.join(categories, img), target_size=(30, 30))
            image = img_to_array(img)
            images.append(image)
            labels.append(category)
    
    return images, labels

def train_SignsModel(data_dir,IMG_HEIGHT = 30,IMG_WIDTH = 30,EPOCHS = 30, save_model = True,saved_model = "saved_model_5_Sign.h5"):
    
    train_path = data_dir + '/datasets'
    
    # Number of Classes
    global NUM_CATEGORIES
    NUM_CATEGORIES = len(os.listdir(train_path))
    print("NUM_CATEGORIES = " , NUM_CATEGORIES)

    # Visualizing all the different Signs
    img_dir = pathlib.Path(train_path)
    plt.figure(figsize=(14,14))
    index = 0
    for i in range(NUM_CATEGORIES):
        plt.subplot(7, 7, i+1)
        plt.grid(False)
        plt.xticks([])
        plt.yticks([])
        print(img_dir)
        sign = list(img_dir.glob(f'{i}/*'))[0]
        img = load_img(sign, target_size=(IMG_WIDTH, IMG_HEIGHT))
        plt.imshow(img)
    plt.show()

    images, labels = load_data(train_path)
    print(len(labels))
    # One hot encoding the labels
    labels = to_categorical(labels)

    # Splitting the dataset into training and test set
    x_train, x_test, y_train, y_test = train_test_split(np.array(images), labels, test_size=0.4)

    #========================================= Model Creation ===============================================
    model = Sequential()
    # First Convolutional Layer
    model.add(Conv2D(filters=16, kernel_size=3, activation='relu', input_shape=(IMG_HEIGHT,IMG_WIDTH,3)))
    model.add(MaxPool2D(pool_size=(2, 2)))
    model.add(Dropout(rate=0.25))
    # Second Convolutional Layer
    model.add(Conv2D(filters=32, kernel_size=3, activation='relu'))
    model.add(MaxPool2D(pool_size=(2, 2)))
    model.add(Dropout(rate=0.25))
    # Flattening the layer and adding Dense Layer
    model.add(Flatten())
    model.add(Dense(units=32, activation='relu'))
    model.add(Dense(NUM_CATEGORIES, activation='softmax'))
    model.summary()
    #========================================================================================================
    
    # Compiling the model
    opt = tf.keras.optimizers.Adam(learning_rate=0.005)
    model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy'])

    # Fitting the model
    history = model.fit(x_train, 
                        y_train,
                        validation_data = (x_test, y_test), 
                        epochs=EPOCHS, 
                        steps_per_epoch=60)


    loss, accuracy = model.evaluate(x_test, y_test)
    print('test set accuracy: ', accuracy * 100)

    accuracy = history.history['accuracy']
    val_accuracy = history.history['val_accuracy']

    loss=history.history['loss']
    val_loss=history.history['val_loss']

    epochs_range = range(EPOCHS)

    plt.figure(figsize=(8, 8))
    plt.subplot(1, 2, 1)
    plt.plot(epochs_range, accuracy, label='Training Accuracy')
    plt.plot(epochs_range, val_accuracy, label='Validation Accuracy')
    plt.legend(loc='lower right')
    plt.title('Training and Validation Accuracy')

    plt.subplot(1, 2, 2)
    plt.plot(epochs_range, loss, label='Training Loss')
    plt.plot(epochs_range, val_loss, label='Validation Loss')
    plt.legend(loc='upper right')
    plt.title('Training and Validation Loss')
    plt.show()
    
    #========================================= Saving Model =================================================
    # save model and architecture to single file
    saved_model_path = os.path.join(data_dir,saved_model)
    if save_model:
        model.save(saved_model_path)
        print("Saved model to disk")
    #========================================================================================================

def EvaluateModelOnImage(model_path,image_path = "",image_label = None):
    # load model
    model = load_model(model_path)
    # summarize model.
    model.summary()
    # load dataset
    if image_path == "":
        def_test_images_path = "self_driving_car_pkg/self_driving_car_pkg/data/test_images"
        # Number of Classes
        global NUM_CATEGORIES
        NUM_CATEGORIES = len(os.listdir(def_test_images_path))
        print("NUM_CATEGORIES = " , NUM_CATEGORIES)

        # Visualizing all the different Signs
        img_dir = pathlib.Path(def_test_images_path)
        plt.figure(figsize=(14,14))
        for i in range(NUM_CATEGORIES):
            ax = plt.subplot(1, 6, i+1)
            plt.tight_layout() 
            plt.grid(False)
            plt.xticks([])
            plt.yticks([])
            sign = list(img_dir.glob(f'{i}/*'))[0]
            img = load_img(sign, target_size=(30, 30))
            image = img_to_array(img)
            images = list()
            images.append(image)
            plt.imshow(img)
            y_pred = model.predict(np.array(images))
            Actual_Vs_Predicted =  'Pred = '+ sign_classes[np.argmax(y_pred)]+ ' \nActual = ' + sign_classes[i]+'\n'
            plt.xlabel(Actual_Vs_Predicted)
        plt.show()
        




    else:    
        # split into input (X) and output (Y) variables
        output = []
        image = load_img(image_path, target_size=(30, 30))
        output.append(np.array(image))
        X_test=np.array(output)

        X = np.array(image).reshape(1,30,30,3)

        if (image_label == 0):
            Y = np.array([[1,0,0,0]])
        elif (image_label == 1):
            Y = np.array([[0,1,0,0]])
        elif (image_label == 2):
            Y = np.array([[0,0,1,0]])
        else:
            Y = np.array([[0,0,0,1]])
            
        print(X.shape)
        print(Y.shape)
        # evaluate the model
        score = model.evaluate(X, Y, verbose=0)
        print("%s: %.2f%%" % (model.metrics_names[1], score[1]*100))

def main():
    if config.Training_CNN:
        train_SignsModel(os.path.abspath("self_driving_car_pkg/self_driving_car_pkg/data/"))
    if Evaluate_Model:
        EvaluateModelOnImage(os.path.abspath("self_driving_car_pkg/self_driving_car_pkg/data/saved_model_5_Sign.h5"))


if __name__ == '__main__':
    main()