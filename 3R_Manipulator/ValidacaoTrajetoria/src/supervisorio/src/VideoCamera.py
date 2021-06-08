#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pyqtgraph.flowchart import Flowchart
from pyqtgraph.Qt import QtGui, QtCore
from collections import deque
from imutils.video import VideoStream
import numpy as np
import pyqtgraph as pg
import math
import random
import rospy
import turtlesim.msg
import sys
import cv2
import argparse
import imutils
import time


distancia = 200
widthCamera = 600
hightCamera = 448
MetrosHorizontal = 53

# ap = argparse.ArgumentParser()
# ap.add_argument("-v", "--video",
# 	help="Caminho pro video")
# ap.add_argument("-b", "--buffer", type=int, default=64,
# 	help="max buffer size")
# args = vars(ap.parse_args())
def CalculaXY(x,y):
    final_x= ((x-widthCamera/2)* MetrosHorizontal)/(widthCamera/2)
    final_y= -1*((y-hightCamera/2)* MetrosHorizontal)/(hightCamera/2)
    return final_x,final_y


def ConstroiPontosAlvo(hsv,Lower, Upper):
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

def ConstroiPontosCentro(hsv,Lower, Upper):
    mask = cv2.inRange(hsv, Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    return cnts,center


def CentroPts(frame,pts_setpoint,cnts,center):
    x1 =0
    y1= 0
    if len(cnts) > 0: 
               
		# encontre o maior contorno da máscara e, em seguida, use
        # para calcular o círculo mínimo e
        # centróide
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #print(radius)
		# só prossiga se o raio atingir um tamanho mínimo
        if radius > 1:
			# desenhe o círculo e o centroide no quadro,
			# em seguida, atualize a lista de pontos rastreados
            cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
	# atualize a fila de pontos
    pts_setpoint.appendleft(center)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    if pts_setpoint[0] is not None:
        x1,y1=CalculaXY(x,y)
        teste = pts_setpoint[0]
        
        cv2.putText(frame, " x:"+ str("%.2f" % x1), 
                        pts_setpoint[0], font,
                        0.8, (0, 255, 255),
                        2, cv2.LINE_AA) 

        cv2.putText(frame, " y:"+ str("%.2f" % y1), 
                        (teste[0],teste[1] +35), font,
                        0.8, (0, 255, 255),
                        2, cv2.LINE_AA)    
        
	
    return pts_setpoint,x1,y1

def AlvoPts(frame,pts,cnts,center,DistanciaCentroX,DistanciaCentroY):
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
            cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
	# atualize a fila de pontos
    pts.appendleft(center)
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    if pts[0] is not None:
        x1,y1=CalculaXY(x,y)
        # calculo para mudar a referencia de centro para o ponto central definido e não a camera.
        x1 = x1 -DistanciaCentroX
        y1 = y1-DistanciaCentroY

               
        # cv2.putText(frame, " x:"+ str("%.2f" % x1), 
        #                 pts[0], font,
        #                 0.8, (0, 255, 255),
        #                 2, cv2.LINE_AA)

        # # fazer com que o texto em 'y' seja impresso um pouco abaixo do x na tela 
        # pts_frame = pts[0] 
        # cv2.putText(frame, " y:"+ str("%.2f" % y1), 
        #                 (pts_frame[0],pts_frame[1] +35), font,
        #                 0.8, (0, 255, 255),
        #                 2, cv2.LINE_AA)    
        
    return pts

class VideoThread(QtCore.QThread):
    change_pixmap_signal = QtCore.pyqtSignal(np.ndarray)

    def run(self):
	blueLower = (35, 140, 60)
	blueUpper = (244, 255, 180)

	greenLower = (6, 88, 96)
	greenUpper = (23, 226, 245)
	pts = deque(maxlen=64)
	pts_setpoint = deque(maxlen=64)
	cap = cv2.VideoCapture(0)
        while True:
	    DistanciaCentroX = 0
	    DistanciaCentroY = 0
		# le o frame atual
	   
            ret, frame = cap.read()
            #frame = cv2.flip(frame,1)
	    frame = imutils.resize(frame, widthCamera)
	    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)   
		
	    #Configração para o setpoint inicial    
	    cnts_setpoint,center_setpoint=ConstroiPontosCentro(hsv,greenLower,greenUpper)
	    pts_setpoint,DistanciaCentroX,DistanciaCentroY=CentroPts(frame,pts_setpoint,cnts_setpoint,center_setpoint)
	    #Configuração  para a garra
	    cnts,center=ConstroiPontosAlvo(hsv,blueLower,blueUpper)
	    pts=AlvoPts(frame,pts,cnts,center,DistanciaCentroX,DistanciaCentroY)
	    for i in range(1, len(pts)):        
			# se algum dos pontos rastreados for Nenhum, ignore
		if pts[i - 1] is None or pts[i] is None:
		    continue
			# caso conwtrário, calcule a espessura da linha e
		# desenhe as linhas de conexão
		#thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 1.5)
		size_line = 10
		cv2.line(frame, pts[i], pts[i-1], (255, 255, 0 ), size_line)
            if ret:
                self.change_pixmap_signal.emit(frame)

