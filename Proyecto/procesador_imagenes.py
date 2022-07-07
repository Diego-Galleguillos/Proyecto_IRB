
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
def centro_color(Solo_color, color_centro):
    ## Convertir a escala de gris
    gray_image = cv2.cvtColor(Solo_color, cv2.COLOR_BGR2GRAY)
    # Convertir a imagen binaria
    ret,thresh = cv2.threshold(gray_image,RANGO_THRESHOLD_1, RANGO_THRESHOLD_2, RANGO_THRESHOLD_3)
    # Busca centro
    cX, cY = encuentra_centro(thresh)
    # Dibuja circulo
    cv2.circle(solo_color, (cX, cY), 5, color_centro, -1)
    return [cX,cY]

# MODOS FALSOS
def modos_falsos():
    global ir_centro, ir_pelota, ir_arco_nuestro, ir_arco_opuesto
    global ir_a_tapar, retroceder, modo_stop, linea_recta, pegar_pelota

    ir_centro = False
    ir_pelota =  False
    ir_arco_nuestro = False
    ir_arco_opuesto = False
    ir_a_tapar = False
    retroceder = False
    modo_stop = False
    linea_recta = False
    pegar_pelota = False
    
    pass
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
COLOR_CENTRO_PELOTA = (255, 255, 255)
COLOR_CENTRO_ROBOT = (68, 104, 248)
COLOR_CENTRO_ENEMIGO = (255, 147, 123)
COLOR_CENTRO_ARCO_1 = (166, 222, 244)
COLOR_CENTRO_ARCO_2 = (251, 61, 14)
COLOR_CENTRO = (255, 255, 255)

COLOR_LINEA_C1_C2 = (68, 104, 248)
COLOR_LINEA_C1_C3 = (255, 255, 255)
COLOR_LETRA_ANGULO = (220, 190, 180)
COLOR_LETRA_MODO = (220, 190, 180)
COLOR_LETRA_DIST = (220, 190, 180)

# TAMAÑOS
TAMANO_LETRA_ANGULO = 0.7
GROSOR_LETRA_ANGULO = 1
GROSOR_LINEA_CENTROS = 2

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

RANGOS_COLOR = np.array([10, 40, 30])

# Activar
CONECTAR = True # activar coneccion arduino
COLORES5 = True # Activar 6 colores


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
pos_interceptar = np.array([0,0])
pos_inicio_tiro = np.array([0,0])

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

ir_centro = True
ir_pelota = False
ir_arco_nuestro = False
ir_arco_opuesto = False
ir_a_tapar = False
retroceder = False
modo_stop = False
linea_recta = False
pegar_pelota = False
disparar = False
disparo = False

# MODOS
modo_espera = True
modo_accion = False
modo_actual = "IR AL CENTRO"

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
        c_color_5 = centro_color(Solo_color_5, COLOR_CENTRO_ENEMIGO)
        c_color_4 = centro_color(Solo_color_4, COLOR_CENTRO_ENEMIGO)
        # Pasa Centro a a array
        v_c_5 = np.array(c_color_5)
        v_c_4 = np.array(c_color_4)
    
    # Detectar y dubuja centro colores
    c_color_3 = centro_color(Solo_color_3, COLOR_CENTRO_PELOTA)
    c_color_2 = centro_color(Solo_color_2, COLOR_CENTRO_ROBOT)
    c_color_1 = centro_color(Solo_color_1, COLOR_CENTRO_ROBOT)
    
    #dibuja centro de cancha y arcos
    cv2.circle(solo_color, pos_centro, 2, COLOR_CENTRO, -1)
    cv2.circle(solo_color, pos_arco_1, 8, COLOR_CENTRO_ARCO_1, -1)
    cv2.circle(solo_color, pos_arco_2, 8, COLOR_CENTRO_ARCO_2, -1)

    # Pasa Centro a a array
    v_c_3 = np.array(c_color_3)
    v_c_2 = np.array(c_color_2)
    v_c_1 = np.array(c_color_1)

    #Calcula posicion para interceptar
    pos_interceptar = v_c_3/2 + pos_arco_1/2
    pos_interceptar = np.array([int(pos_interceptar[0]), int(pos_interceptar[1])])

    #Calcula posicion para pegar
    pos_inicio_tiro = v_c_3 + 70*(v_c_3 - pos_arco_2)/np.linalg.norm((v_c_3 - pos_arco_2))
    pos_inicio_tiro = np.array([int(pos_inicio_tiro[0]), int(pos_inicio_tiro[1])])

    
    # Calcula angulo y vector entre B y C
    if ir_pelota:
        alpha, d = angulo(v_c_2, v_c_3, v_c_1)
        if MODO_ANTERIOR == "PEGARLE A LA PELOTA":
            if abs(alpha) <= 15 and np.linalg.norm(d/4.5) < 13:
                pegar_pelota = False
                ir_pelota = False
                MODO_ANTERIOR = "IR A LA PELOTA"
                ir_arco_opuesto = True
        if MODO_ANTERIOR == "DISPARAR LA PELOTA":
            if abs(alpha) <= 15 and np.linalg.norm(d/4.5) < 13:
                pegar_pelota = False
                disparo = True
                MODO_ANTERIOR = "IR A LA PELOTA"
    elif ir_centro:
        alpha, d = angulo(v_c_2, pos_centro, v_c_1)
    elif ir_arco_nuestro:
        alpha, d = angulo(v_c_2, pos_arco_1, v_c_1)
    elif ir_arco_opuesto:
        alpha, d = angulo(v_c_2, pos_arco_2, v_c_1)
    elif ir_a_tapar:
        alpha, d = angulo(v_c_2, pos_interceptar, v_c_1)
        cv2.circle(solo_color, pos_interceptar, 8, COLOR_CENTRO, -1)
    elif pegar_pelota:
        alpha, d = angulo(v_c_2, pos_inicio_tiro, v_c_1)
        cv2.circle(solo_color, pos_inicio_tiro, 10, (255, 255, 0), -1)
        if np.linalg.norm(d/3) <= 10:
            pegar_pelota = False
            ir_pelota = True
            MODO_ANTERIOR = "PEGARLE A LA PELOTA"
    elif disparar:
        alpha, d = angulo(v_c_2, pos_inicio_tiro, v_c_1)
        cv2.circle(solo_color, pos_inicio_tiro, 10, (255, 255, 0), -1)
        if np.linalg.norm(d/3) <= 10:
            pegar_pelota = False
            ir_pelota = True
            MODO_ANTERIOR = "DISPARAR LA PELOTA"
   
    ## Dibuja linea entre centros

    
    if ir_pelota:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, c_color_3, COLOR_LINEA_C1_C3, GROSOR_LINEA_CENTROS)
    elif ir_centro:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, pos_centro, COLOR_LINEA_C1_C3, GROSOR_LINEA_CENTROS)
    elif ir_arco_nuestro:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, pos_arco_1, COLOR_LINEA_C1_C3, GROSOR_LINEA_CENTROS)
    elif ir_arco_opuesto:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, pos_arco_2, COLOR_LINEA_C1_C3, GROSOR_LINEA_CENTROS)
    elif ir_a_tapar:
        solo_color = cv2.line(solo_color, c_color_1, c_color_2, COLOR_LINEA_C1_C2, GROSOR_LINEA_CENTROS)
        solo_color = cv2.line(solo_color, c_color_1, pos_interceptar, COLOR_LINEA_C1_C3, GROSOR_LINEA_CENTROS)        

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
        solo_color, f"MODO: = {modo_actual}", 
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
        ir_centro = True
        ir_pelota = False
        ir_arco_nuestro = False
        ir_arco_opuesto = False
        modo_stop = False
        modo_actual = "IR AL CENTRO"
        ir_a_tapar = False
        linea_recta = False
        disparar = False
        
    elif tecla & 0xFF == ord('p'): #ir a la pelota
        ir_centro = False
        ir_pelota = True
        modo_stop = False
        modo_actual = "IR A LA PELOTA"
        ir_a_tapar = False
        linea_recta = False
        disparar = False

    elif tecla & 0xFF == ord('a'): #ir al arco opuesto
        ir_centro = False
        ir_pelota = False
        ir_arco_nuestro = False
        ir_arco_opuesto = True
        modo_stop = False
        modo_actual = "IR A LA ARCO OPUESTO"
        ir_a_tapar = False
        linea_recta = False
        disparar = False

    elif tecla & 0xFF == ord('d'): #ir a nuestro arco
        ir_centro = False
        ir_pelota = False
        ir_arco_nuestro = True
        ir_arco_opuesto = False
        modo_stop = False
        modo_actual = "IR A NUESTRO ARCO"
        ir_a_tapar = False
        linea_recta = False
        disparar = False
    
    elif tecla & 0xFF == ord('r'): #retroceder
        retroceder = True
        MODO_ANTERIOR = modo_actual
        modo_stop = False
        modo_actual = "RETROCEDER"
        linea_recta = False
        disparar = False

    elif tecla & 0xFF == ord('b'): #borrar colores
        nClick = 1
        nClick_2 = 1

    elif tecla & 0xFF == ord('s'): # Stop 
        modo_stop = True
        modo_espera = True
        ir_centro = False
        ir_pelota = False
        ir_arco_nuestro = False
        ir_arco_opuesto = False
        MODO_ANTERIOR = modo_actual
        modo_actual = "STOP"
        ir_a_tapar = False
        linea_recta = False
        disparar = False

    elif tecla & 0xFF == ord('t'): #tapar el arco
        ir_centro = False
        ir_pelota = False
        ir_arco_nuestro = False
        ir_arco_opuesto = False
        modo_stop = False
        ir_a_tapar = True
        linea_recta = False
        modo_actual = "IR A TAPAR ARCO"
        disparar = False

    elif tecla & 0xFF == ord('w'):

        ir_centro = False
        ir_pelota = False
        ir_arco_nuestro = False
        ir_arco_opuesto = False
        modo_stop = False
        ir_a_tapar = False
        linea_recta = True
        modo_actual = "LINEA RECTA"
        disparar = False
    
    elif tecla & 0xFF == ord('q'): #Posicionas, y pegar a la pelota 

        ir_centro = False
        ir_pelota = False
        ir_arco_nuestro = False
        ir_arco_opuesto = False
        modo_stop = False
        ir_a_tapar = False
        linea_recta = False
        pegar_pelota = True
        disparar = False
        modo_actual = "PEGARLE A LA PELOTA"

    elif tecla & 0xFF == ord('e'): #Posicionas, y dispara la pelota 

        ir_centro = False
        ir_pelota = False
        ir_arco_nuestro = False
        ir_arco_opuesto = False
        modo_stop = False
        ir_a_tapar = False
        linea_recta = False
        pegar_pelota = False
        disparar = True
        modo_actual = "DISPARAR LA PELOTA"

    # Mandar informacion arduino

    if Listo and CONECTAR:
        if modo_accion:
                if retroceder:
                    com_serial.distancia = -40
                    com_serial.angulo = 0
                    time.sleep(2)
                    retroceder = False
                    modo_actual = MODO_ANTERIOR
                
                elif disparo:
                    com_serial.distancia = 200
                    com_serial.angulo = 0
                    time.sleep(0.8)
                    com_serial.distancia = 0
                    time.sleep(0.25)                
                    disparar = False
                    disparo = False
                    ir_centro = True 
                    
                elif modo_stop:

                    com_serial.distancia = 0
                    com_serial.angulo = 0
                    retroceder = False
                    modo_actual = "STOP"
                    
                elif linea_recta:
                    com_serial.distancia = 70 
                    com_serial.angulo = 0

                else:
                    if np.linalg.norm(d/3) >= 15 or modo_actual != 'IR A LA PELOTA':
                        com_serial.distancia = int(np.linalg.norm(d/1.5))
                    else:
                        com_serial.distancia = 0
                    try:
                        com_serial.angulo = int(np.degrees(alpha))
                    except:
                        pass

        elif modo_espera:

            if ir_centro:
                modo_accion = True
            elif ir_pelota:
                modo_accion = True
            elif ir_arco_nuestro:
                modo_accion = True
            elif ir_arco_opuesto:
                modo_accion = True
            elif pegar_pelota:
                modo_accion = True

    

com_serial.distancia = 0
com_serial.angulo = 0
cap.release()
cv2.destroyAllWindows()