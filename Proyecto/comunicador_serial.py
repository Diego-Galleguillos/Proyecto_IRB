import threading
import time

class Comunicacion_serial(threading.Thread): # Hereda de Thread
    
    # Atributo de clase
    # Accesible mediante Clase.lock o self.lock desde una instancia
 
    def __init__(self, servidor, distancia_inicial, daemon = True):
        super().__init__(name=servidor)
        self.servidor = servidor
        self.distancia = distancia_inicial
        self.angulo = 0
        self.angulo_anterior = 0
        self.distancia_anterior = 1
    
    def run(self):
        print("Esta funcionando...")
        while True:
            msg2 = str(self.distancia) + ","+ str(self.angulo)+ '\n'
            print("mensaje: ", msg2)
            #if self.distancia != self.distancia_anterior: 
            self.servidor.write(self.funcion_mensaje(msg2))
            print(f"distancia: {self.distancia}")
            self.distancia_anterior = self.distancia
            time.sleep(0.4)
            
                
    
    def funcion_mensaje(self, msgOn):
        msgOn = msgOn
        # El Ambos mensajes que estan en formato Sring deben ser transformados en un arreglo de bytes mediante la funcion .encode
        msgncode = str.encode(msgOn) 
        return msgncode
