
##################################################
#                    IMPORTS                     #
##################################################

# Externos
import numpy as np
import cv2
import time
import serial
from threading import Thread
import numpy as np
import serial


# Propios
from comunicador_serial import Comunicacion_serial

##################################################
#              DEFINIMOS FUNCIONES               #
##################################################

# imprime mensaje seleccion
def imprime_color_elegido(color_sel, x, y, nr):
    print("Color", str(nr) +  ": ", color_sel )
    print("Eje x: ", x)
    print("Eje y: ", y)
 
# devuelve color seleccionado (e imprime)
def color_seleccionado(hsv_frame, x, y, nr):
    color_sel = hsv_frame[y,x]
    imprime_color_elegido(color_sel_1, x, y, nr)
    return color_sel

# Definimos clicks
def _mouseEvent(event, x, y, flags, param):
#COLOR 1 es el centro, color 2 el segundo color del robot y color 3 la pelota
    global nClick, Listo, restart
    global color1_hsv
    global color2_hsv
    global color_sel_1, color_sel_2, color_sel_3, color_sel_4, color_sel_5, color_sel_6
    global nClick_2
    global pos_arco_1, pos_arco_2
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        if(nClick == 1):
            color_sel_1 = color_seleccionado(hsv_frame, x, y, nClick)
            nClick += 1

        elif(nClick == 2):
            color_sel_2 = color_seleccionado(hsv_frame, x, y, nClick)
            nClick += 1

        elif(nClick == 3):
            color_sel_3 = color_seleccionado(hsv_frame, x, y, nClick)
            nClick += 1
            if not(COLORES5):
                Listo = True

        elif(nClick == 4 and COLORES5):
            color_sel_4 = color_seleccionado(hsv_frame, x, y, nClick)
            nClick += 1

        elif(nClick == 5 and COLORES5):
            color_sel_5 = color_seleccionado(hsv_frame, x, y, nClick)
            nClick += 1
            Listo = True

        elif(nClick == 4 or nClick == 6):
            print("PROXIMO CLICK REINICIA COLORES")
            nClick += 1

        else:
            Listo = False
            nClick= 1

    if event == cv2.EVENT_RBUTTONDOWN:

        if nClick_2 == 1:
            nClick_2 += 1
            print("Se selcciono arco 1")
            pos_arco_1 = np.array([x, y])
        elif nClick_2 == 2:
            nClick_2 += 1
            print("Se selcciono arco 1")
            pos_arco_2 = np.array([x, y])
        if nClick_2 == 3:
            nClick_2 = 1
            print("SIGUIENTE CLICK DERECHO SELECCIONA ARCO")
    """
    if event == cv2.EVENT_:
        escojer_arco = True
        print("Seleccione los arcos")
    """

# Calucula angulo
def angulo(A, B, C):
	# C es el centro, B es el balon
	a = B-C
	b = A-C
    # Se calcula angulo
	alpha = np.arccos(a.dot(b)/(np.linalg.norm(a)*np.linalg.norm(b)))
	alphasen = np.arcsin(np.cross(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)))
    # Define angulo
	if alphasen <0:
		return -alpha, a
	return alpha, a

# Calcula centro
def encuentra_centro(thresh):
    # Calcular con momentes
    M = cv2.moments(thresh)
    # calculate x,y coordinate of center
    if M["m00"] == 0:
        cX = 1
        cY = 1
    else:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    return cX, cY

# Dibuja y detecta centro objeto 
def centro_color(Solo_color):
    ## Convertir a escala de gris
    gray_image = cv2.cvtColor(Solo_color, cv2.COLOR_BGR2GRAY)
    # Convertir a imagen binaria
    ret,thresh = cv2.threshold(gray_image,RANGO_THRESHOLD_1, RANGO_THRESHOLD_2, RANGO_THRESHOLD_3)
    # Busca centro
    cX, cY = encuentra_centro(thresh)
    # Dibuja circulo
    cv2.circle(solo_color, (cX, cY), 5, COLOR_CENTRO, -1)
    return [cX,cY]
##################################################
#              DEFINIMOS PARAMETROS              #
##################################################

COM = "COM9 "
BANDRATE = 38400 

# Nombres
WINDOW_NAME_1 = 'frame1'
WINDOW_NAME_2 = 'frame2'

# Camara
NCAM = 1 # Camara a utilizar
HZ_CAMARA = 1 # 1 = 50hz y 2 = 60 hz

# COLORES
COLOR_CENTRO = (255, 255, 255)
COLOR_LINEA_C1_C2 = (255, 255, 255)
COLOR_LINEA_C1_C3 = (255, 255, 255)
COLOR_LETRA_ANGULO = (220, 190, 180)
COLOR_LETRA_MODO = (220, 190, 180)
COLOR_LETRA_DIST = (220, 190, 180)

# TAMAÑOS
TAMANO_LETRA_ANGULO = 0.7
GROSOR_LETRA_ANGULO = 1
GROSOR_LINEA_CENTROS = 3

TAMANO_LETRA_MODO = 0.7
TAMANO_LETRA_DIST = 0.7 
GROSOR_LETRA_MODO = 1
GROSOR_LETRA_DIST = 1
GROSOR_LINEA_MODO = 3


# POSICION
POS_TEXTO_ANGULO = (20, 85)
POS_TEXTO_MODO = (20, 50)
POS_TEXTO_DIST = (20, 120)

# RANGOS
RANGO_THRESHOLD_1 = 50
RANGO_THRESHOLD_2 = 255
RANGO_THRESHOLD_3 = 0

RANGOS_COLOR = np.array([10, 40, 40])

# Activar
CONECTAR = True # activar coneccion arduino
COLORES5 = False # Activar 6 colores


##################################################
#              DEFINIMOS VARIABLES               #
##################################################
distancia_inicial = 0
nClick = 1
nClick_2 = 1
Listo = False
restart = True
pos_arco_1 = np.array([0,0])
pos_arco_2 = np.array([0,0])
pos_centro = np.array([0,0])

## Vectores creados para el mouse_event() ##
color1_hsv = np.array([0,0,0])
color2_hsv = np.array([0,0,0])

## Limites colores

color_sel_1 = np.array([0, 0, 0])
color_sel_2 = np.array([0, 0, 0])
color_sel_3 = np.array([0, 0, 0])
color_sel_4 = np.array([0, 0, 0])
color_sel_5 = np.array([0, 0, 0])
color_sel_6 = np.array([0, 0, 0])

# Objetivos
IR_CENTRO = True
IR_PELOTA = False
IR_ARCO_NUESTRO = False
IR_ARCO_OPUESTO = False
RETROCEDER = False
MODO_STOP = False

# MODOS 
MODO_ESPERA = True
MODO_ACCION = False 
MODO_ACTUAL = "IR AL CENTRO"

##################################################
#                    Programa                    #
##################################################

# Iniciamos cumunicacion con Arduino 
if CONECTAR:
    servidor = serial.Serial(COM, baudrate = BANDRATE, timeout = 1)
    com_serial = Comunicacion_serial(servidor, distancia_inicial)
    com_serial.start()

# Conecta camara
cap = cv2.VideoCapture(NCAM) # Establece coneccion camara
cap.set(cv2.CAP_PROP_SETTINGS,HZ_CAMARA) # Define frames camara


# Creamos pestañas
cv2.namedWindow(WINDOW_NAME_1) # Pestaña 1
cv2.moveWindow(WINDOW_NAME_1, 50, 100)
cv2.namedWindow(WINDOW_NAME_2) # Pestaña 2
cv2.moveWindow(WINDOW_NAME_2, 700, 100) 



while(True):
    
    ret, frame = cap.read() #
    image_size = frame.shape
    ancho = int(frame.shape[1])/2 
    altura = int(frame.shape[0])/2
    pos_centro = np.array([int(ancho), int(altura)])
    # Pasar a de HSV
    imagen_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )

    # Reconocer colores
    color_1 = cv2.inRange(imagen_hsv, color_sel_1 - RANGOS_COLOR, color_sel_1 + RANGOS_COLOR)
    color_2 = cv2.inRange(imagen_hsv, color_sel_2 - RANGOS_COLOR, color_sel_2 + RANGOS_COLOR)
    color_3 = cv2.inRange(imagen_hsv, color_sel_3 - RANGOS_COLOR, color_sel_3 + RANGOS_COLOR)
    
    if COLORES5:
        color_4 = cv2.inRange(imagen_hsv, color_sel_4 - RANGOS_COLOR, color_sel_4 + RANGOS_COLOR)
        color_5 = cv2.inRange(imagen_hsv, color_sel_5 - RANGOS_COLOR, color_sel_5 + RANGOS_COLOR)
    
    # Mostrar pestana 1 (a color normal)
    cv2.imshow(WINDOW_NAME_1,frame)
    cv2.setMouseCallback(WINDOW_NAME_1, _mouseEvent)
    mask1 = cv2.bitwise_or(color_2, color_3)
    mask = cv2.bitwise_or(mask1, color_1)
    solo_color = cv2.bitwise_and(frame, frame, mask= mask)
    Solo_color_3 = cv2.bitwise_and(frame, frame, mask= color_3)
    Solo_color_2 = cv2.bitwise_and(frame, frame, mask= color_2)
    Solo_color_1 = cv2.bitwise_and(frame, frame, mask= color_1)

    
    if COLORES5:
        Solo_color_5 = cv2.bitwise_and(frame, frame, mask= color_5)
        Solo_color_4 = cv2.bitwise_and(frame, frame, mask= color_4)
        # Detectar y dubuja centro colores
        c_color_5 = centro_color(Solo_color_5)
        c_color_4 = centro_color(Solo_color_4)
        # Pasa Centro a a array
        v_c_5 = np.array(c_color_5)
        v_c_4 = np.array(c_color_4)
    
    # Detectar y dubuja centro colores
    c_color_3 = centro_color(Solo_color_3)
    c_color_2 = centro_color(Solo_color_2)
    c_color_1 = centro_color(Solo_color_1)

    #dibuja centro de cancha y arcos
    cv2.circle(solo_color, pos_centro, 8, COLOR_CENTRO, -1)
    cv2.circle(solo_color, pos_arco_1, 8, COLOR_CENTRO, -1)
    cv2.circle(solo_color, pos_arco_2, 8, COLOR_CENTRO, -1)

    # Pasa Centro a a array
    v_c_3 = np.array(c_color_3)
    v_c_2 = np.array(c_color_2)
    v_c_1 = np.array(c_color_1)

    # Calcula angulo y vector entre B y C
    if IR_PELOTA:
        alpha, d = angulo(v_c_2, v_c_3, v_c_1)
    elif IR_CENTRO:
        alpha, d = angulo(v_c_2, pos_centro, v_c_1)
    elif IR_ARCO_NUESTRO:
        alpha, d = angulo(v_c_2, pos_arco_1, v_c_1)
    elif IR_ARCO_OPUESTO:
        alpha, d = angulo(v_c_2, pos_arco_2, v_c_1)

   
    ## Dibuja linea entre centros

    
    if IR_PELOTA:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, c_color_3, COLOR_LINEA_C1_C3, GROSOR_LINEA_CENTROS)
    elif IR_CENTRO:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, pos_centro, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
    elif IR_ARCO_NUESTRO:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, pos_arco_1, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
    elif IR_ARCO_OPUESTO:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, pos_arco_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)

    ## Incertar Texto
    solo_color = cv2.putText(
        solo_color, f"Angulo = {np.degrees(alpha)}", 
        POS_TEXTO_ANGULO, 
        cv2.FONT_HERSHEY_SIMPLEX, 
        TAMANO_LETRA_ANGULO, 
        COLOR_LETRA_ANGULO, 
        GROSOR_LETRA_ANGULO, 
        cv2.LINE_AA)
    

    ## Modo funcion 
    solo_color = cv2.putText(
        solo_color, f"MODO: = {MODO_ACTUAL}", 
        POS_TEXTO_MODO, 
        cv2.FONT_HERSHEY_SIMPLEX, 
        TAMANO_LETRA_MODO, 
        COLOR_LETRA_ANGULO, 
        GROSOR_LETRA_ANGULO, 
        cv2.LINE_AA)

        ## Modo funcion 
    solo_color = cv2.putText(
        solo_color, f"Distancia: = {com_serial.distancia}", 
        POS_TEXTO_DIST, 
        cv2.FONT_HERSHEY_SIMPLEX, 
        TAMANO_LETRA_DIST, 
        COLOR_LETRA_DIST, 
        GROSOR_LETRA_DIST, 
        cv2.LINE_AA)

    cv2.imshow(WINDOW_NAME_2,solo_color)
    ## Cada 1 frame revisa si hay alguna tecla apretada.
    ## 0xFF guarda los primeros 8 bits de tecla, ya que tiene 32. 
    
    tecla = cv2.waitKey(1) 
    if tecla & 0xFF == 27: #
        break
    elif tecla & 0xFF == ord('c'): #ir al centro
        IR_CENTRO = True
        IR_PELOTA = False
        IR_ARCO_NUESTRO = False
        IR_ARCO_OPUESTO = False
        MODO_STOP = False
        MODO_ACTUAL = "IR AL CENTRO"
        
    elif tecla & 0xFF == ord('p'): #ir a la pelota
        IR_CENTRO = False
        IR_PELOTA = True
        MODO_STOP = False
        MODO_ACTUAL = "IR A LA PELOTA"
    elif tecla & 0xFF == ord('a'): #ir al arco opuesto
        IR_CENTRO = False
        IR_PELOTA = False
        IR_ARCO_NUESTRO = False
        IR_ARCO_OPUESTO = True
        MODO_STOP = False
        MODO_ACTUAL = "IR A LA ARCO OPUESTO"

    elif tecla & 0xFF == ord('d'): #ir a nuestro arco
        IR_CENTRO = False
        IR_PELOTA = False
        IR_ARCO_NUESTRO = True
        IR_ARCO_OPUESTO = False
        MODO_STOP = False
        MODO_ACTUAL = "IR A NUESTRO ARCO"
    
    elif tecla & 0xFF == ord('r'): #Retroceder
        RETROCEDER = True
        MODO_ANTERIOR = MODO_ACTUAL
        MODO_STOP = False
        MODO_ACTUAL = "RETROCEDER"

    elif tecla & 0xFF == ord('b'): #borrar colores
        nClick = 1
        nClick_2 = 1

    elif tecla & 0xFF == ord('s'): # Stop 
        MODO_STOP = True
        MODO_ESPERA = True
        IR_CENTRO = False
        IR_PELOTA = False
        IR_ARCO_NUESTRO = False
        IR_ARCO_OPUESTO = False
        MODO_ANTERIOR = MODO_ACTUAL
        MODO_ACTUAL = "STOP"

    # Mandar informacion arduino

    if Listo and CONECTAR:
        if MODO_ACCION:
                if RETROCEDER:
                    com_serial.distancia = -40
                    com_serial.angulo = 0
                    time.sleep(2)
                    RETROCEDER = False
                    MODO_ACTUAL = MODO_ANTERIOR
                    
                elif MODO_STOP:

                    com_serial.distancia = 0
                    com_serial.angulo = 0
                    RETROCEDER = False
                    MODO_ACTUAL = "STOP"
                else:
                    if np.linalg.norm(d) > 8:
                        com_serial.distancia = int(np.linalg.norm(d/4.3))
                    else:
                        com_serial.distancia = 0
                    try:
                        com_serial.angulo = int(np.degrees(alpha))
                    except:
                        pass

        elif MODO_ESPERA:

            if IR_CENTRO:
                MODO_ACCION = True
            elif IR_PELOTA:
                MODO_ACCION = True
            elif IR_ARCO_NUESTRO:
                MODO_ACCION = True
            elif IR_ARCO_OPUESTO:
                MODO_ACCION = True

    

com_serial.distancia = 0
cap.release()
cv2.destroyAllWindows()