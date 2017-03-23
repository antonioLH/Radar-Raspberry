#!/usr/bin/python
# -*- coding: utf-8 -*-

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
import datetime
import cv2

# Calculamos el tiempo transcurrido
def secs_diff(endTime, begTime):
    diff = (endTime - begTime).total_seconds()
    return diff    

# Calculo de la velocidad
def get_speed(pixels, ftperpixel, secs):
    if secs > 0.0:
        return ((pixels * ftperpixel)/ secs) * 3.6
    else:
        return 0.0

# Distancia desde la cámara al objeto	
DISTANCE = 2
# El numero de pixeles que hace falta que cambien para tenerlo en cuenta
THRESHOLD = 50
# El tamaño minimo del objeto para tenerlo en cuenta
MIN_AREA = 175
BLURSIZE = (15,15)
# Tamaño imagen
IMAGEWIDTH = 640
IMAGEHEIGHT = 480
RESOLUTION = [IMAGEWIDTH,IMAGEHEIGHT]
FPS = 30
WAITING = 0
TRACKING = 1
SAVING = 2
UNKNOWN = 0
LEFT_TO_RIGHT = 1
RIGHT_TO_LEFT = 2

state = WAITING
direction = UNKNOWN
initial_x = 0
last_x = 0

# Este es el cálculo del ángulo de la cámara viene definido para las cámaras de raspberry
FOV = 53.5
frame_width_ft = 2*(math.tan(math.radians(FOV*0.5))*DISTANCE)
ftperpixel = frame_width_ft / float(IMAGEWIDTH)

#-- other values used in program
base_image = None
abs_chg = 0
kph = 0
secs = 0.0
show_bounds = True
showImage = True
ix,iy = -1,-1
fx,fy = -1,-1
drawing = False
setup_complete = False
tracking = False
text_on_image = 'Sin movimiento'
loop_count = 0


# Inicializamos la camara
camera = PiCamera()
camera.resolution = RESOLUTION
camera.framerate = FPS

rawCapture = PiRGBArray(camera, size=camera.resolution)
# Echamos una foto inicial
camera.capture(rawCapture, format="bgr", use_video_port=True)
image = rawCapture.array
rawCapture.truncate(0)
# Generamos la imagen blanca completa
org_image = image.copy()

# Comienza el grueso del código, lo que vamos ha hacer es comenzar una captura continua de la cámara
# Cuando detectemos un cambio en un frame respecto a la imagen inicial procederemos con los cálculos

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # Cogemos el tiempo actual
    timestamp = datetime.datetime.now()
 
    # Cogemos los 3 colores 
    image = frame.array
    gray = image
    # Pasamos la imagen a escala de grises, que lo que hace es asignarle un numer entre 0-255
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, BLURSIZE, 0)
 
    # Si no se ha definido la base de la imagen, es decir esta echando fotos y pasandolas a blanco
    if base_image is None:
        base_image = gray.copy().astype("float")
        lastTime = timestamp
        rawCapture.truncate(0)
        continue
 
    # Ahora comparamos las imagenes para saber cuantos pixeles han cambiado entre la base imagen y la continua
    frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(base_image))
    thresh = cv2.threshold(frameDelta, THRESHOLD, 255, cv2.THRESH_BINARY)[1]
  
    # Dilatamos la imagen para encontrar contornos
    thresh = cv2.dilate(thresh, None, iterations=2)
    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    # Buscamos movimiento 
    motion_found = False
    biggest_area = 0
 
    # Vamos a encontrar los contornos de esta imagen para ver si hay movimiento
    for c in cnts:
        (x, y, w, h) = cv2.boundingRect(c)
        # Aproximación del tamaño del objeto
        found_area = w*h 
        # Ver si supera nuestro limite
        if (found_area > MIN_AREA) and (found_area > biggest_area):  
            biggest_area = found_area
            motion_found = True

	#Si el area era grande por tanto hay movimiento
    if motion_found:
        #Si esta en espera se cambia a detección
        if state == WAITING:
            # Inicializamos la detección
            state = TRACKING
	    # Obtenemos la posición y el tiempo
            initial_x = x
            last_x = x
            initial_time = timestamp
            last_kph = 0
            text_on_image = 'Detectando'
            print(text_on_image)
        else:
            #Si está en modo detección obtenemos la dirección del objeto y calculamos los cambios
            # y los mostramos por consola
            if state == TRACKING: 
                #Si va de izquierda a derecha
                if x >= last_x:
                    direction = LEFT_TO_RIGHT
                    abs_chg = x + w - initial_x
                #Si va de derecha a izquierda
                else:
                    direction = RIGHT_TO_LEFT
                    abs_chg = initial_x - x
                # Calculamos la velocidad a partir de los cambios en la imagen    
                secs = secs_diff(timestamp,initial_time)
                kph = get_speed(abs_chg,ftperpixel,secs)
                #Imprimimos por consola
                print("--> chg={}  secs={}  km/h={}".format(abs_chg,secs,"%.0f" % kph))
                # Una vez el objeto ha salido del área, guardamos
                if ((x <= 2) and (direction == RIGHT_TO_LEFT)) \
                        or ((x+w >= 440 - 2) \
                        and (direction == LEFT_TO_RIGHT)):
                    # Guardamos la foto obtenida que más cambios tenga
                    cv2.imwrite("Foto.jpg",image)
                    state = SAVING
                # Una vez salga el objeto, obtener la última velocidad y última posición
                last_kph = kph
                last_x = x
    else:
        #Si no encontramos movimiento, si el estado no es de espera lo actualizamos y mostramos
        # que no hay movimiento
        if state != WAITING:
            state = WAITING
            direction = UNKNOWN
            text_on_image = 'Sin detección'
            print(text_on_image)

    # Si el objeto está parado, no lo contamos como movimiento e impedimos
    # que la cámara este detectándolo
    if (state == WAITING) or (loop_count > 20):    

        state=WAITING;
        loop_count = 0
         
    # Limpiamos el stream
    rawCapture.truncate(0)
    loop_count = loop_count + 1
