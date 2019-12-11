#----------------------S-TABLE-----------------------
#--------------------CONTROL PID---------------------
#----------------------------------------------------
# importar librerias necesarias
from collections import deque
import numpy as np
import argparse
import cv2
import RPi.GPIO as GPIO 
from simple_pid import PID
import time
from pylab import *
import matplotlib.pyplot as plt
#Se crear una funcion para poder escalar la salida para los servos
def mapeo(inp, inmin, inmax, omin, omax):
    return (((inp - inmin) / (inmax - inmin))*(omax - omin)) + omin;
#configuracion de GPIO
GPIO.setmode(GPIO.BOARD)   
GPIO.setup(40,GPIO.OUT) #Se utiliza el pin 40 para el eje x
GPIO.setup(38,GPIO.OUT) #Se utiliza el pin 38 para el eje y
sx = GPIO.PWM(40,50)
sy = GPIO.PWM(38,50)  
sx.start(7)
sy.start(7)
#Se establece la resolucion de la camara (3:4)
re1=20 #Resolucion en x
re2=int(re1*3/4) #Resolucion en y
#Se establece valores iniciales
pidx=7
pidy=7
x=int(re1/2)
y=int(re2/2)
t1 = 0
x1, y1, y2, px, py = [], [], [],[],[]
#PID
#Se establece el setpoint del PID
setpx=10
setpy=7
#Se crea con la libreria simple_pid el pid para cada eje
pid1 = PID(0.35, 0.1, 0.2, setpoint=setpx)
pid2 = PID(0.35, 0.1, 0.2, setpoint=setpy)
#Se establecen sus limites de salida
pid1.output_limits = (-10, 10)
pid2.output_limits=(-10,10)
#Se establece el tiempo de muestreo
pid1.sample_time = 0.01
pid2.sample_time=0.01
pid1.proportional_on_measurement = False
pid2.proportional_on_measurement = False

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
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV
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
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            #Se convierte lo detectado a la resolucion establec	da
            x=round(((x/160)*re1),1)
            y=round(((y/120)*re2),1)
            #------------------Control--------------------------------
            #Se escala la salida del PID para la salida de los servos
            pidy=mapeo(-pid2(int(y)), -10, 10, 4.5, 10.5)
            pidx=mapeo(-pid1(int(x)), -10, 10, 4.5, 10.5)
            #Se imprimen valores en consola
            print("pidx: ", pidx, " pidy: ",pidy," x:",x," y:",y)
            #Se escribe en el servo
            sx.ChangeDutyCycle(pidx)
            sy.ChangeDutyCycle(pidy)
            #Se llenan los vectores para los graficos
            x1.append(t1)
            y1.append(setpx)
            y2.append(setpy)
            px.append(x)
            py.append(y)
            t1 += 1
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
