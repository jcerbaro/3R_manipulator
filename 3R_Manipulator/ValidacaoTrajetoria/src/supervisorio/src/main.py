#!/usr/bin/env python
# -*- coding: utf-8 -*-
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
distancia = 100
widthCamera = 600
hightCamera = 337
MetrosHorizontal = 1.34
MetrosVertical = 0.79


def CalculaXY(x,y):
    final_x= (((x-widthCamera)* MetrosHorizontal)/(widthCamera))
    final_y= (((hightCamera-y)* MetrosVertical)/(hightCamera))
    return final_x,final_y



ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="Caminho pro video")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

def ConstroiPontosAlvo(Lower, Upper):
    # # constroi mascara para a cor escolhida e executa alguns ajustes de dilatação e erosão para remover defeitos na imagem.
    mask = cv2.inRange(hsv, Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # encontra o contorno do item e inicializa o (x,y) no centro
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    return cnts,center

def ConstroiPontosCentro(Lower, Upper):
    mask = cv2.inRange(hsv, Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    return cnts,center


def CentroPts(cnts,center):
    x1 =0
    y1= 0
    if len(cnts) > 0: 
               
		# encontre o maior contorno da máscara e, em seguida, use
        # para calcular o círculo mínimo e
        # centróide
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        print(x,y)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        center_T = (int((M["m10"] / M["m00"]) + ((0.30* widthCamera)/MetrosHorizontal)), hightCamera- int((M["m01"] / M["m00"]) +((0.22* widthCamera)/MetrosHorizontal)))
        #print(center_T)
		# só prossiga se o raio atingir um tamanho mínimo
        if radius > 1:
			# desenhe o círculo e o centroide no quadro,
			# em seguida, atualize a lista de pontos rastreados
            #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            axesLength = (int((0.10* widthCamera)/MetrosHorizontal), int((0.10* hightCamera)/MetrosVertical))
            centerTX = int(M["m10"] / M["m00"] + ((0.36 * widthCamera)/MetrosHorizontal))
            centerTY = int(M["m01"] / M["m00"] - ((0.18 * hightCamera)/MetrosVertical))
            angle = 180  
            startAngle = 0            
            endAngle = 360
            color = (255, 0, 0)
            thickness = 2
            cv2.ellipse(frame, (centerTX,centerTY), axesLength,angle, startAngle, endAngle, color, thickness)
	# atualize a fila de pontos
    pts_setpoint.appendleft(center)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    
    if pts_setpoint[0] is not None:
        x1,y1=CalculaXY(x,y)
        teste = pts_setpoint[0]
	
    return pts_setpoint,x1,y1

def AlvoPts(cnts,center,DistanciaCentroX,DistanciaCentroY):
    if len(cnts) > 0: 
               
		# encontre o maior contorno da máscara e, em seguida, use
        # para calcular o círculo mínimo e
        # centróide
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		# só prossiga se o raio atingir um tamanho mínimo
        if radius > 1:
			# desenhe o círculo e o centroide no quadro,
			# em seguida, atualize a lista de pontos rastreados
            #cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
	# atualize a fila de pontos
    pts.appendleft(center)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    if pts[0] is not None:
        x1,y1=CalculaXY(x,y)
        # calculo para mudar a referencia de centro para o ponto central definido e não a camera.
        x1 = x1 -DistanciaCentroX
        y1 = y1-DistanciaCentroY
        
        cv2.putText(frame, " x:"+ str("%.2f" % x1), 
                        pts[0], font,
                        0.8, (0, 255, 255),
                        2, cv2.LINE_AA)

        # fazer com que o texto em 'y' seja impresso um pouco abaixo do x na tela 
        pts_frame = pts[0] 
        cv2.putText(frame, " y:"+ str("%.2f" % y1), 
                        (pts_frame[0],pts_frame[1] +35), font,
                        0.8, (0, 255, 255),
                        2, cv2.LINE_AA)    
        
    return pts



#define cor que o programa
# irá seguir low e upper 
blueLower = (45, 60, 60)
blueUpper = (70, 255, 255)

greenLower = (6, 88, 96)
greenUpper = (23, 226, 245)
pts = deque(maxlen=args["buffer"])
pts_setpoint = deque(maxlen=args["buffer"])
# se não tem video usa webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()
else:
    vs = cv2.VideoCapture(args["video"])

time.sleep(2.0)

while True:
    DistanciaCentroX = 0
    DistanciaCentroY = 0
	# le o frame atual
    frame = vs.read()
    #frame = cv2.flip(frame,1)
    frame = frame[1] if args.get("video", False) else frame
	# se chega no final do video
    if frame is None:
        break
	# redimensiona e converte para HSV (tipo de espaço de cor utilizado)
    frame = imutils.resize(frame, widthCamera)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)   
	
    #Configração para o setpoint inicial    
    cnts_setpoint,center_setpoint=ConstroiPontosCentro(greenLower,greenUpper)
    pts_setpoint,DistanciaCentroX,DistanciaCentroY=CentroPts(cnts_setpoint,center_setpoint)
    #Configuração  para a garra
    cnts,center=ConstroiPontosAlvo(blueLower,blueUpper)
    pts=AlvoPts(cnts,center,DistanciaCentroX,DistanciaCentroY)
    for i in range(1, len(pts)):        
		# se algum dos pontos rastreados for Nenhum, ignore
        if pts[i - 1] is None or pts[i] is None:
            continue
		# caso conwtrário, calcule a espessura da linha e
        # desenhe as linhas de conexão
        #thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 1.5)
        size_line = 2
        cv2.line(frame, pts[i], pts[i-1], (255, 255, 0 ), size_line)
    



	# mostre o quadro na nossa tela
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
	# se a tecla 'q' for pressionada, pare o loop
    if key == ord("q"):
        break
# se não estivermos usando um arquivo de vídeo, pare o stream de vídeo da câmera
if not args.get("video", False):
    vs.stop()
else:
    vs.release()
cv2.destroyAllWindows()



