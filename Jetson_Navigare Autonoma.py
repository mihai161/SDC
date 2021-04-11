#importa librarii
import RPi.GPIO as GPIO
import cv2
import time
import numpy as np
import tensorflow as tf
from tensorflow import keras 
import pickle5 as pickle
import tensorflow_hub as hub
from tensorflow import keras
from tensorflow.keras.models import model_from_json
from tensorflow.keras.applications.mobilenet_v2 import MobileNetV2

numeFata = 'Fata'
numarFata = 0
numeStanga = 'Stanga'
numarStanga = 0
numeDreapta = 'Dreapta'
numarDreapta = 0
jpg = '.jpg'
rxPin18 = 18
rxPin16 = 16
width = 224
height = 224
dim = (width, height)

#Initializeaza pinii pentru comunicarea cu Arduino
GPIO.setmode(GPIO.BOARD)
GPIO.setup(rxPin18, GPIO.OUT)
GPIO.setup(rxPin16, GPIO.OUT)

#reconstrucite model 
model = tf.keras.Sequential([
  hub.KerasLayer("https://tfhub.dev/google/tf2-preview/mobilenet_v2/feature_vector/4", 
                 output_shape=[1280],
                 trainable=False),
  tf.keras.layers.Dropout(0.4),
  tf.keras.layers.Dense(3, activation='softmax')
])
model.build([None, 224, 224, 3])

#Incarca modeleul antrenat 
model.load_weights("SDC_Conv.h5")

vcLeft = cv2.VideoCapture(1, cv2.CAP_V4L) # Incaraca captura video de la camera 1
vcRight = cv2.VideoCapture(0,cv2.CAP_V4L) # Incaraca captura video de la camera 2
num_disp = 112 - 16

# Initializeaza Obiectul stereo
stereo = cv2.StereoBM_create(numDisparities = num_disp, blockSize = 15)

#Setari pentru captura de adancime
stereo.setMinDisparity(16)

stereo.setNumDisparities(num_disp)

stereo.setBlockSize(15)

stereo.setDisp12MaxDiff(5)

stereo.setUniquenessRatio(15)

stereo.setSpeckleRange(34)

stereo.setSpeckleWindowSize(100)

#Steaza pinii pe LOW pentru a nu trimite miscari nedorite
GPIO.output(rxPin16, GPIO.LOW)
GPIO.output(rxPin18, GPIO.LOW)

if vcLeft.isOpened() and vcRight.isOpened():
        rvalLeft, frameLeft = vcLeft.read()
        rvalRight, frameRight = vcRight.read()

else:
        rvalLeft = False
        rvalRight = False

while rvalLeft and rvalRight: # Daca camerele sunt pornite

        #Cardele sunt mai intai capturate si apoi decodate pentru a fi 
        #cat mai putina diferenta intre cele doua
        vcLeft.grab() # Citeste captura video de la camera 1
        vcRight.grab() # Citeste captura video de la camera 2

        rvalLeft, frameLeft = vcLeft.retrieve() # Decodeaza captura video de la camera 1

        rvalRight, frameRight = vcRight.retrieve() # Decodeaza captura video de la camera 2

        #Intoarce camera 2
        flippedRightFrame = cv2.flip(frameRight, 0)

        frFlippedRightFrame = cv2.flip(flippedRightFrame, 1)

        #Transforma cardrele in greyscale (alb-negru)
        frameLeftNew = cv2.cvtColor(frameLeft, cv2.COLOR_BGR2GRAY)

        frameRightNew = cv2.cvtColor(frFlippedRightFrame, cv2.COLOR_BGR2GRAY)

        #Procesare cadru de adancime
        disparity = stereo.compute(frameLeftNew, frameRightNew).astype(np.float32) / 16.0

        disp_map = (disparity - 16)/num_disp

        # Transforma imaginile in dimensiunea potrivita
        disp_map = cv2.resize(disp_map, dim, interpolation = cv2.INTER_AREA)

        gray = disp_map

        img2 = np.zeros((1, np.array(disp_map).shape[0], np.array(disp_map).shape[1], 3))

        img2[:,:,:,0] = gray
        img2[:,:,:,1] = gray
        img2[:,:,:,2] = gray

        # Prezice directia in care trebuie sa vireze bazat
        # pe captura de adancime din acest moment
        modelPred = model.predict(img2)
        modelPred = modelPred.tolist()
        modelPred = modelPred[0]

        # Trimite comenzile de navigare catre Arduino in functie de predicite
        if modelPred[0] >= modelPred[1] and modelPred[0] >= modelPred[2]:
                GPIO.output(rxPin16, GPIO.LOW)
                GPIO.output(rxPin18, GPIO.HIGH)
                print('Dreapta')

        elif modelPred[1] >= modelPred[0] and modelPred[1] >= modelPred[2]:
                GPIO.output(rxPin16, GPIO.HIGH)
                GPIO.output(rxPin18, GPIO.HIGH)
                print('Fata')

        elif modelPred[2] >= modelPred[0] and modelPred[2] >= modelPred[1]:
                GPIO.output(rxPin16, GPIO.HIGH)
                GPIO.output(rxPin18, GPIO.LOW)
                print('Stanga')
        else:
                GPIO.output(rxPin16, GPIO.LOW)
                GPIO.output(rxPin18, GPIO.LOW)		
               	print('Stop')

        key = cv2.waitKey(20)

        if key == 27:

                break

# Opreste camerele
vcLeft.release()

vcRight.release()
