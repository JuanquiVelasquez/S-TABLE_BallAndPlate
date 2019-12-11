#----------------------S-TABLE-----------------------
#--------------------CONTROL LQR---------------------
#----------------------------------------------------
# importar librerias necesarias
from collections import deque
import numpy as np
import argparse
import cv2
import RPi.GPIO as GPIO 
import time
from pylab import *
import matplotlib.pyplot as plt
import control
import math
#Se crear una funcion para poder escalar la salida para los servos
def mapeo(inp, inmin, inmax, omin, omax):
    return (((inp - inmin) / (inmax - inmin))*(omax - omin)) + omin;
#Configuramos los servos
GPIO.setmode(GPIO.BOARD)   
GPIO.setup(40,GPIO.OUT)
GPIO.setup(38,GPIO.OUT)
sx = GPIO.PWM(40,50)
sy = GPIO.PWM(38,50)  
sx.start(7)
sy.start(7)
#Se establece la resolucion de la camara (3:4)
re1=40 #Resolucion en x
re2=int(re1*3/4) #Resolucion en y
#Se establece valores iniciales
tc=0
cua=400
g=5
pidx=7
pidy=7
xold=0;
yold=0;
servx=7.5
servy=7.5
stold=0
vxold=0
vyold=0
ux=0
uy=0
x=int(re1/2)
y=int(re2/2)
sppx=-5
sppy=0
t1 = 0
ta=0
x1, y1, y2, px, py = [], [], [],[],[]
#LQR
#Se establece el setpoint inicial del LQR
setpx=20
setpy=15
#Ganancia K del control LQR
Kx=([3.16,3.3])
Ky=([3.16,3.3])

#-------------VIsion----------------
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())
#Se define el color de la pelota
colorLower = np.array([0, 100, 0])
colorUpper = np.array([179, 255, 255])
pts = deque(maxlen=args["buffer"])

if not args.get("video", False):
    camera = cv2.VideoCapture(0)

else:
    camera = cv2.VideoCapture(args["video"])
# Se establece la resolucion de la camara (160x120)
camera.set(3,160)
camera.set(4,120)
#Se inicia el ciclo de captura
while True:
    (grabbed, frame) = camera.read()
    if args.get("video") and not grabbed:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        #Se obtiene la posicion en x, y de la pelota
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #Se establece un radio minimo de deteccion
        if radius > 5:
            st=time.time()
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            #Se convierte lo detectado a la resolucion establecida y se obtienen la
            #nueva posicion x, y de la pelota
            x=round(((x/160)*re1),1)
            y=round(((y/120)*re2),1)
            #------------------Control--------------------------------
            #Ley de Control LQR u=-K*x
            #Donde x[1]=[(posicion),(velocidad)]
            #Para establecer el setpoint se establece de esta manera
            #x(posicion)=(setpoint-posicion_actual), las constantes se establecieron debido
            #a la respuesta del sistema real
            #x(velocidad)=(0-velocidad), 0 debido a que la velocidad en el setpoint para estar
            #estatico debe ser 0
            ux=-(Kx[0]*((setpx)-x+8.5)+Kx[1]*(0-(x-xold)/(st-stold)))
            uy=-(Ky[0]*((setpy)-y)+Ky[1]*(0-(y-yold)/(st-stold)))
            #Se escala la salida del LQR para la salida de los servos
            servx=mapeo(float(ux), -Kx[0]*re1, Kx[0]*re1, 4.5, 10.5)
            servy=mapeo(float(uy), -Ky[0]*re2, Ky[0]*re2, 4.5, 10.5)
            #se pone limites para los servos
            if servx<=4.5:
                servx=4.5
            if servy<=4.5:
                servy=4.5
            #Se escribe en los servos
            sx.ChangeDutyCycle(servx)
            sy.ChangeDutyCycle(servy)
            #Se llenan los vectores para los graficos
            x1.append(t1)
            y1.append(setpx)
            y2.append(setpy)
            px.append(x)
            py.append(y)
            #Se varia el setpoint en un determinado tiempo
            #creando la figura de un cuadrado
            if tc<=cua/4 and tc>=0:
                setpx=10
                setpy=10*3/4
            if tc>cua/4 and tc<=cua/2:
                setpx=30
                setpy=10*3/4
            if tc>cua/2 and tc<=cua*3/4:
                setpx=30
                setpy=30*3/4
            if tc>cua*3/4 and tc<cua:
                setpx=10
                setpy=30*3/4
            if tc==cua:
                tc=0
            tc += 1
            ta+=4
            t1 += 1
            xold=x
            yold=y
            stold=st
    pts.appendleft(center)      
    #Se muestra la imagen capturada por la camara
    cv2.imshow("Frame", frame)
    #Se muestra la mascara capturada por la camara
    cv2.imshow("Mask", mask)
    key = cv2.waitKey(1) & 0xFF
    #si se presiona q se termina el proceso
    if key == ord("q"):
        break
    
#Detenemos la captura de la camara
camera.release()
#Detenemos los servos
sx.ChangeDutyCycle(7)
time.sleep(0.5)
sy.ChangeDutyCycle(7)
time.sleep(0.5)
sx.stop()
sy.stop()
GPIO.cleanup() 
#Se grafican los valores
plt.subplot(2, 1, 1)
plt.plot(x1, px, '-', lw=2)

plt.xlabel('time (s)')
plt.ylabel('x')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(x1, py, '-', lw=2)

plt.xlabel('time (s)')
plt.ylabel('y')
plt.grid(True)

plt.subplot(2, 1, 1)
plt.plot(x1, y1, '-', lw=2)

plt.xlabel('time (s)')
plt.ylabel('x')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(x1, y2, '-', lw=2)

plt.xlabel('time (s)')
plt.ylabel('y')
plt.grid(True)
#Se muestran los valores
plt.tight_layout()
plt.show()

#Cerramos todas las ventanas
cv2.destroyAllWindows()

