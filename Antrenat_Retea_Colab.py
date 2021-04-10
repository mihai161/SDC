from google.colab import drive
drive.mount('/content/drive') #initializat Google drive (aici sunt toate datele)

#importat librarii
import os
from shutil import copy2
import csv
from __future__ import absolute_import, division, print_function, unicode_literals

try:
  %tensorflow_version 2.x
except Exception:
  pass
import tensorflow as tf
import os
import numpy as np
import matplotlib.pyplot as plt
import tensorflow_hub as hub
import pandas as pd
pd.set_option("display.precision", 8)

#locatia imaginilor pentru antrenare
data_root = ("/content/drive/My Drive/SDC Dataset/Traning Images")

#marimea imaginilor in pixeli
IMAGE_SHAPE = (224, 224)
TRAINING_DATA_DIR = str(data_root)

#impartire de date pentru antrenat si validat
datagen_kwargs = dict(rescale=1./255, validation_split=.20)
#Obiect pentru imaginile de validare
valid_datagen = tf.keras.preprocessing.image.ImageDataGenerator(**datagen_kwargs)
valid_generator = valid_datagen.flow_from_directory(
    TRAINING_DATA_DIR, 
    subset="validation", 
    shuffle=True,
    target_size=IMAGE_SHAPE
)
#Obiect pentru imaginile de antrenat
train_datagen = tf.keras.preprocessing.image.ImageDataGenerator(**datagen_kwargs)
train_generator = train_datagen.flow_from_directory(
    TRAINING_DATA_DIR, 
    subset="training", 
    shuffle=True,
    target_size=IMAGE_SHAPE)

print (train_generator.class_indices) #Output clase:{'Dreapta': 0, 'Fata': 1, 'Stanga': 2}

labels = '\n'.join(sorted(train_generator.class_indices.keys()))

#Preluat etichete pentru clase
with open('labels.txt', 'w') as f:
  f.write(labels)
  
IMAGE_SIZE = 224

#Creare instanta de model
model.build([None, 224, 224, 3])

model.summary() Output:  """Model: "sequential"
                            _________________________________________________________________
                            Layer (type)                 Output Shape              Param #   
                            =================================================================
                            keras_layer (KerasLayer)     (None, 1280)              2257984   
                            _________________________________________________________________
                            dropout (Dropout)            (None, 1280)              0         
                            _________________________________________________________________
                            dense (Dense)                (None, 3)                 3843      
                            =================================================================
                            Total params: 2,261,827
                            Trainable params: 3,843
                            Non-trainable params: 2,257,984
                            _________________________________________________________________"""

  #Optimizatore Adam pentru a eficientiza modelul
optimizer = tf.keras.optimizers.Adam(lr=1e-3)
  
model.compile(
optimizer=optimizer,
loss='categorical_crossentropy',
metrics=['acc'])
  
steps_per_epoch = np.ceil(train_generator.samples/train_generator.batch_size)
val_steps_per_epoch = np.ceil(valid_generator.samples/valid_generator.batch_size)

#Incepe procesul de antrenat pe 100 de "epoci" Durata: ~4 ore
hist = model.fit(
    train_generator, 
    epochs=100,
    verbose=1,
    steps_per_epoch=steps_per_epoch,
    validation_data=valid_generator,
    validation_steps=val_steps_per_epoch).history
