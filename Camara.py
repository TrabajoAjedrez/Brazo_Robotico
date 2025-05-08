#Importar librerías necesarias
import cv2
import numpy as np
import time
import serial

# Capturar video desde la webcam (cambia el 0 por la URL si usas ESP32-CAM)
cap = cv2.VideoCapture(1)  # 0 = primera cámara conectada
if not cap.isOpened(): #Verificar si la cámara se abrió correctamente
    print("No se pudo abrir la cámara")
    exit()

# Inicializar serie al Arduino/ESP32 que mueve el stepper
arduino = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2)  # espera a que la placa esté lista

# Rango de color para detectar la arena (HSV) (Dtecta mejor la arena que el RGB)qçç
#lower_sand = np.array([15, 40, 100])
#upper_sand = np.array([35, 255, 255])

lower_green = np.array([35, 50, 50]) 
upper_green = np.array([85, 255, 255])

ret, tmp = cap.read() 
alto,ancho = tmp.shape[:2] # Obtener el ancho de la imagen (frame) capturada
zona_izq = ancho / 3 # Dividir la imagen en 3 partes
zona_der = 2 * ancho / 3 

while True:
    ret, frame = cap.read()  #La funcion read() devuelve dos valores: un booleano (1 si lee el frame correctamente) y el frame capturado en formato BGR
    if not ret:
        print("No se pudo leer el frame")
        break

    cv2.line(frame, (int(zona_izq), 0), (int(zona_izq), alto), (255, 0, 255), 2)
    cv2.line(frame, (int(zona_der), 0), (int(zona_der), alto), (255, 0, 255), 2)

    # Convertir frame de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Crear máscara para el color de la arena. Basicamente se le dice a la cámara que ignore los colores que no están dentro del rango de colores que pones como lower y upper.
    # Es decir dibuja en blanco los pixeles dentro de ese rango y en negro los que estan fuera.
    #mask = cv2.inRange(hsv, lower_sand, upper_sand)

    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Encontrar contornos en la máscara
    # findContours devuelve dos valores, una lista de contornos encontrados, y la jerarquia de los mismo, pero la jerarquia no nos interas por eso se pone _
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #RETR_EXTERNAL contornos externoS, CHAIN_APPROX_SIMPLE simplifica los puntos del contorno para optimizar el cálculo.
    alineado = False #Flag para detectar cuando la imagen este centrada sobre la arena

    for cnt in contours: #En C++ es como colocar for(int i=0; i<contours.size();i++) {auto cnt = contours[i];}
        area = cv2.contourArea(cnt) #Devuelve cuantos pizeles tiene el contorno, sirve para eliminar ruido
        if area > 1000:  # Filtrar objetos muy pequeños
            # Calcular centroide
            M = cv2.moments(cnt) #Moments devuelve un diccionario
            #Nos interesan: m00=area del contorno, m10=sumatoria de x*intensidad y m01=sumatoria de y*intensidad, que son los momentos de primer orden, que nos permiten calcular el centroide del contorno.
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"]) #Esta operacion nos da el centroide del contorno, es decir la coordenada x del centroide.
                cy = int(M["m01"] / M["m00"]) #Esta operacion nos da el centroide del contorno, es decir la coordenada y del centroide.
                # Dibujar círculo en el centro y contorno
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1) #Imagen, coordenada, radio, color, grosor
                texto = f"X: {cx}, Y: {cy}" #Texto a mostrar en la imagen
                cv2.putText(frame, texto, (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2) #Imagen, texto, coordenada, fuente, tamaño de la fuente, color, grosor
                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2) #Imagen, contorno, -1 = todos los contornos, color, grosor

                #Lógica de zonas y envío de comando al stepper
                if cx < zona_izq:
                    cmd = "GIRA_IZQ\n"
                elif cx > zona_der:
                    cmd = "GIRA_DER\n"
                else:
                    cmd = "PARAR\n"
                    alineado = True

                #Enviar comando al Arduino
                #arduino.write(cmd.encode())

                if alineado:
                    break  # sal del for de contornos

    # Mostrar la imagen en una ventana
    cv2.imshow("Vista de la cámara", frame)
    cv2.imshow("Máscara de arena", mask)

    # Salir al presionar 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'): #El oxFF es una mascara para obtener el valor ASCII del caracter en cualquier sistema operativo, waitKey(1) espera 1 milisegundo para detectar si se presiona una tecla
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
