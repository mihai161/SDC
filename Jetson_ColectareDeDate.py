#Functia acestui cod este de a crea o baza de date 
#pentru a antrena reteaua neuronala

#importa librarii
import RPi.GPIO as GPIO
import cv2
import time
import numpy as np


masa = 3
numeFata = 'Fata'
numarFata = 0
numeStanga = 'Stanga'
numarStanga = 0
numeDreapta = 'Dreapta'
numarDreapta = 0
jpg = '.jpg'
rxPin1 = 18
rxPin2 = 16

#Initializeaza pinii pentru comunicarea cu Arduino
GPIO.setmode(GPIO.BOARD)
GPIO.setup(rxPin1, GPIO.IN)
GPIO.setup(rxPin2, GPIO.IN)

vcLeft = cv2.VideoCapture(1) # Incaraca captura video de la camera 1
vcRight = cv2.VideoCapture(0) # Incaraca captura video de la camera 2

firstTime = time.time() # Referinta timp

totalFramesPassed = 0 # Numarul cardelor trecute

treshhold = np.array([])

if vcLeft.isOpened() and vcRight.isOpened():
        rvalLeft, frameLeft = vcLeft.read() 
        rvalRight, frameRight = vcRight.read() 

else:
        rvalLeft = False
        rvalRight = False

num_disp = 112 - 16
stereo = cv2.StereoBM_create(numDisparities = num_disp, blockSize = 23) # Initializeaza Obiectul stereo

#Setari pentru captura de adancime
stereo.setMinDisparity(8)

stereo.setNumDisparities(num_disp)

stereo.setBlockSize(23)

stereo.setDisp12MaxDiff(5)

stereo.setUniquenessRatio(15)

stereo.setSpeckleRange(22)

stereo.setSpeckleWindowSize(100)

while rvalLeft and rvalRight: # Daca camerele sunt porinte

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

        # Citeste valorile primite de la Arduino
        rxPin1State = GPIO.input(rxPin1)
        rxPin2State = GPIO.input(rxPin2)

        #Verifica in ce directie merge roverul in acest moment
        #si salveaza cardul cu eticheta corespunzatoare
        if rxPin1State == 1 and rxPin2State == 0:
                numeDreapta = numeDreapta + str(numarDreapta) + jpg
                cv2.imwrite(numeDreapta, 255 * disp_map) 
                numeDreapta = 'Dreapta'
                numarDreapta = numarDreapta + 1
                print(numeDreapta)
                print("Dreapta")

        if rxPin1State == 1 and rxPin2State == 1:
                numeStanga = numeStanga + str(numarStanga) + jpg
                cv2.imwrite(numeStanga, 255 * disp_map)
                numeStanga = 'Stanga' 
                numarStanga = numarStanga + 1
                print(numeStanga)
                print("Stanga")

        if rxPin1State == 0 and rxPin2State == 1:
                numeFata = numeFata + str(numarFata) + jpg
                cv2.imwrite(numeFata, 255 * disp_map)
                numeFata = 'Fata'  
                numarFata = numarFata + 1
                print(numeFata)
                print("Fata")

        if rxPin1State == 0 and rxPin2State == 0:
                print("Stop")
	
        disp_map = cv2.rotate(disp_map,cv2.ROTATE_180)		

        #Arata cadrele de adancime
        cv2.imshow("Disparity", disp_map)

        key = cv2.waitKey(20)

        if key == 27:

                break

# Opreste camerele
vcLeft.release()

vcRight.release()
