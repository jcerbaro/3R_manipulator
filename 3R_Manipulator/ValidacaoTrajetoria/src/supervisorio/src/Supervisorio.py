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
from VideoCamera import *
from Terminal import *
from std_msgs.msg import String
from custom_msg.msg import set_angles,status_arm
import threading
import transforms3d

# def AddTime (): 
#   threading.Timer(0.25, AddTime).start ()
#   print "teste"

abregarra = True
def PublicaAngulo(junta):
    pub = rospy.Publisher('/cmd_3R', set_angles,queue_size=10)
    msg = set_angles()
    if junta == "t":
        msg.reset = False
        msg.retry = False
        msg.emergency_stop = False
        msg.set_OMB = spin0.value()
        msg.set_COT = spin1.value()
        msg.set_PUN = spin2.value()
        pub.publish(msg)
    if junta == "g":
        msg.reset = False
        msg.retry = False
        msg.emergency_stop = False
        if abregarra == False:
            msg.set_GAR = False
            abregarra = False
        else:
            msg.set_GAR == True
            abregarra = True
        pub.publish(msg)
    if junta == "rt":
        msg.reset = False
        msg.retry = True
        msg.emergency_stop = False
        msg.set_OMB = spin0.value()
        msg.set_COT = spin1.value()
        msg.set_PUN = spin2.value()
        pub.publish(msg)
    if junta == "rs":
        msg.reset = True
        msg.retry = False
        msg.emergency_stop = False
        msg.set_OMB = -40
        msg.set_COT = 145
        msg.set_PUN = -133
        pub.publish(msg)
    if junta == "st":
        msg.reset = False
        msg.retry = False
        msg.emergency_stop = True
        msg.set_OMB = spin0.value()
        msg.set_COT = spin1.value()
        msg.set_PUN = spin2.value()
        pub.publish(msg)

def update_image(cv_img):
    qt_img = convert_cv_qt(cv_img)
    image_label.setPixmap(qt_img)
    
def convert_cv_qt( cv_img):
    rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb_image.shape
    bytes_per_line = ch * w
    convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
    p = convert_to_Qt_format.scaled(300, 300, QtCore.Qt.KeepAspectRatio)
    return QtGui.QPixmap.fromImage(p)

#Declara e define informações da GUI.
app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.setWindowTitle('Supervisor')
mw.resize(800,800)
cw = QtGui.QWidget()
l = QtGui.QGridLayout()
l2 = QtGui.QGridLayout()
l_btn = QtGui.QGridLayout()
vbox = QtGui.QGridLayout()

mw.setCentralWidget(cw)

l.addLayout(l2,0,0)
l.addLayout(vbox,1,0)
l2.addLayout(l_btn,3,1)

image_label = QtGui.QLabel()
image_label.resize(600, 480)

l.addWidget(image_label,1,2)

# set the vbox layout as the widgets layout
#thread = VideoThread()
#thread.change_pixmap_signal.connect(update_image)
#thread.start()

#adiciona terminal
# terminal = Terminal() 
# vbox.addWidget(terminal)

spins = [ 
    ("Ombro", pg.SpinBox(value=-40, bounds=[-40, None])),
    ("Cotovelo", pg.SpinBox(value=145, bounds=[None, 145])),
    ("Punho", pg.SpinBox(value=-133, bounds=[-133, None])),
    
]


labels = []

countSpin = 0
for text, spin in spins:
    label = QtGui.QLabel(text)
    labels.append(label)
    l2.addWidget(label,countSpin,0)
    globals()["spin"+str(countSpin)] = spin
    l2.addWidget((globals()["spin"+str(countSpin)]),countSpin,1)
    countSpin += 1



#Cria os widgets de botão.
btn3 = QtGui.QPushButton('Garra!')
btn4 = QtGui.QPushButton('Enviar')
btn5 = QtGui.QPushButton('Retry!')
btn6 = QtGui.QPushButton('Reset!')
btn7 = QtGui.QPushButton('Parada!')

#eventos


btn3.clicked.connect(lambda: PublicaAngulo("g"))
btn4.clicked.connect(lambda: PublicaAngulo("t"))
btn5.clicked.connect(lambda: PublicaAngulo("rt"))
btn6.clicked.connect(lambda: PublicaAngulo("rs"))
btn7.clicked.connect(lambda: PublicaAngulo("st"))


l_btn.addWidget(btn3,0,0)
l_btn.addWidget(btn4,0,1)
l_btn.addWidget(btn5,0,2)
l_btn.addWidget(btn6,0,3)
l_btn.addWidget(btn7,0,4)
 

# video_frame = QtGui.QLabel()
# l_video.addWidget(video_frame,2,1)

#Cria os widgets de gráficos.
pw1 = pg.PlotWidget(name='Ombro')
pw2 = pg.PlotWidget(name='Cotovelo')
pw3 = pg.PlotWidget(name='Punho')
pw4 = pg.PlotWidget(name='Joint 4')



#Indica que o layout é no formato de grid e posiciona os widgets.
#l.addWidget(fc.widget(), 0, 0, 3, 1) 
l.addWidget(pw1,0,1)
l.addWidget(pw2,1,1)
l.addWidget(pw3,2,1)
l.addWidget(pw4,0,2)



#Seta o layout montado e abre a janela.
cw.setLayout(l)

mw.show()

#Seta as séries dos gráficos e inicializa com valores quaisquer.
J1_setpoint = pw1.plot(pen=pg.mkPen('r', width=3), name="Setpoint")
J1_data = pw1.plot(pen=pg.mkPen('b', width=3), name="Data")

J2_setpoint = pw2.plot(pen=pg.mkPen('r', width=3), name="Setpoint")
J2_data = pw2.plot(pen=pg.mkPen('b', width=3), name="Data")

J3_setpoint = pw3.plot(pen=pg.mkPen('r', width=3), name="Setpoint")
J3_data = pw3.plot(pen=pg.mkPen('b', width=3), name="Data")

J4_setpoint = pw4.plot(pen=pg.mkPen('r', width=3), name="Setpoint")
J4_data = pw4.plot(pen=pg.mkPen('b', width=3), name="Data")

pw1.setLabel('left', 'Ombro', units='Graus')
pw1.setLabel('bottom', 'Time', units='ms')
pw1.setYRange(-2*math.pi, 2*math.pi)

pw2.setLabel('left', 'Cotovelo', units='Graus')
pw2.setLabel('bottom', 'Time', units='ms')
pw2.setYRange(-2*math.pi, 2*math.pi)

pw3.setLabel('left', 'Punho', units='Graus')
pw3.setLabel('bottom', 'Time', units='ms')
pw3.setYRange(-2*math.pi, 2*math.pi)

pw4.setLabel('left', 'Posição X', units='m')
pw4.setLabel('bottom', 'Posição Y', units='m')


#Inicializa as séries dos gráficos como lista.
data_J1x = []
data_J1y = []
data_J2x = []
data_J2y = []
data_J3x = []
data_J3y = []

#graficos do pulso contados
data_J1x_contados = []
data_J1y_contados = []
data_J2x_contados = []
data_J2y_contados = []
data_J3x_contados = []
data_J3y_contados = []

data_contados_tx = []
data_contados_ty = []
def callback_OMB(data):     
    now_data = rospy.Time.now()    
    timestamp = (now_data-now_start)/1000000
    data_J1x.append(int(str(timestamp)))
    data_J1y.append(data.pulsos_setpoint/200 + (-40))
    data_J1x_contados.append(int(str(timestamp)))
    data_J1y_contados.append(data.pulsos_contados/200 + (-40))

def callback_COT(data):
    now_data = rospy.Time.now()    
    timestamp = (now_data-now_start)/1000000
    data_J2x.append(int(str(timestamp)))
    data_J2y.append(data.pulsos_setpoint/325 +(145))  
    data_J2x_contados.append(int(str(timestamp)))
    data_J2y_contados.append(data.pulsos_contados/325 +(145))


def callback_PUN(data):    
    now_data = rospy.Time.now()    
    timestamp = (now_data-now_start)/1000000
    data_J3x.append(int(str(timestamp)))
    data_J3y.append(data.pulsos_setpoint/148 +(-133))
    data_J3x_contados.append(int(str(timestamp)))
    data_J3y_contados.append(data.pulsos_contados/148 +(-133))

def EulerOrientation(ombro,cotovelo,punho):
    #Posição da base do manipulador, sendo o formato: P_init = [x;y;z;1].
    Base = [0,0,0,1]
    #Parâmetros D-H do manipulador.
    alpha = [0,             0,             0           ]
    theta = [np.deg2rad(ombro),  np.deg2rad(cotovelo),  np.deg2rad(punho)]
    a     = [0.235,            0.245,            0.065          ]
    d     = [0,             0,             0           ]
    #print((ombro) +  (cotovelo)+  (punho))
    #Matrizes de transformação homogênea de cada junta.
    T = np.zeros((4,4,3))
 
    for i in range(3):
        
        T[:,:,i] = [[np.cos(theta[i]),     -np.cos(alpha[i])*np.sin(theta[i]),       np.sin(alpha[i])*np.sin(theta[i]),     a[i]*np.cos(theta[i])],
                    [np.sin(theta[i]),      np.cos(alpha[i])*np.cos(theta[i]),      -np.sin(alpha[i])*np.cos(theta[i]),     a[i]*np.sin(theta[i])],
                    [0,                  np.sin(alpha[i]),                     np.cos(alpha[i]),                   d[i]              ],
                    [0,                  0,                                 0,                               1                ]]
    

    #Matriz de transformação homogênea de todo o manipulador.
    #print(T)
    
    #H = T[:,:,0]*T[:,:,1]*T[:,:,2]
    t12 = np.matmul(T[:,:,0],T[:,:,1])
    H = np.matmul(t12,T[:,:,2])
    
    #Cálculo do vetor de posição final do efetuador:
    P_final = H*Base
    
    #Posição cartesiana do efetuador.
    Effector_position_XYZ = P_final[0:2,3]#posição x y

    #Matriz de rotação do efetuador. Neste exemplo ela pode ser extraída diretamente do método sendo aplicado.
    Effector_Rotmat = H[0:2,0:2]
    #Ângulos de Euler da orientação do efetuador.
    #Rotation_ZYX = transforms3d.euler.mat2euler(Effector_Rotmat,axes='rzyx'); 
    Rotation_ZYX = np.arctan2(H[1,0],H[0,0])# orientação, plotar em caixinha
    return Effector_position_XYZ,Rotation_ZYX

#Atualiza os gráficos.
WTvar = 0
Lenvar = 0
WSvar = 0
def CalculaWindowsSize(WT,data):
    global WTvar
    global Lenvar
    global WSvar
    if len(data) > 0:
        if data[len(data)-1] >= WT + WTvar:
            WTvar = data[len(data)-1]
            WS = len(data) - Lenvar
            WSvar = WS
            Lenvar = len(data)
            return WS
        else:
            return WSvar
    

def updateData():
    window_time = 90000 # um segundo
    window_size = 10000 # para os outros
    window_size_j1 = CalculaWindowsSize(window_time,data_J1x)
    window_size_j2 = CalculaWindowsSize(window_time,data_J2x)
    window_size_j3 = CalculaWindowsSize(window_time ,data_J3x) *2

    set_point_OMB_x = data_J1x[len(data_J1x) -window_size_j1:len(data_J1x)]
    set_point_OMB_y  = data_J1y[len(data_J1y) -window_size_j1:len(data_J1y)]    
    pulsos_contados_OMB_x = data_J1x_contados[(len(data_J1x_contados) -window_size_j1):len(data_J1x_contados)]
    pulsos_contados_OMB_y = data_J1y_contados[(len(data_J1y_contados) -window_size_j1):len(data_J1y_contados)]

    set_point_COT_x = data_J2x[len(data_J2x) -window_size_j2:len(data_J2x)]
    set_point_COT_y  = data_J2y[len(data_J2y) -window_size_j2:len(data_J2y)]    
    pulsos_contados_COT_x = data_J2x_contados[(len(data_J2x_contados) -window_size_j2):len(data_J2x_contados)]
    pulsos_contados_COT_y = data_J2y_contados[(len(data_J2y_contados) -window_size_j2):len(data_J2y_contados)]

    set_point_PUN_x = data_J3x[len(data_J3x) -window_size_j3:len(data_J3x)] 
    set_point_PUN_y  =  data_J3y[len(data_J3y) -window_size_j3:len(data_J3y)]     
    pulsos_contados_PUN_x = data_J3x_contados[(len(data_J3x_contados) -window_size_j3):len(data_J3x_contados)]
    pulsos_contados_PUN_y = data_J3y_contados[(len(data_J3y_contados) -window_size_j3):len(data_J3y_contados)]
    
    #ombro:
   
    J1_setpoint.setData(x=set_point_OMB_x, y=set_point_OMB_y)
    J1_data.setData(x=pulsos_contados_OMB_x, y=pulsos_contados_OMB_y)
    pw1.setXRange(set_point_OMB_x[0], set_point_OMB_x[window_size_j1-1])
    pw1.setYRange(-50, 20)

	#Cotovelo.
    J2_setpoint.setData(x=set_point_COT_x, y=set_point_COT_y)
    J2_data.setData(x=pulsos_contados_COT_x, y=pulsos_contados_COT_y)
    pw2.setXRange(set_point_COT_x[0], set_point_COT_x[window_size_j2-1])
    pw2.setYRange(40, 130)

    
	#Punho.
    J3_setpoint.setData(x=set_point_PUN_x, y=set_point_PUN_y)
    J3_data.setData(x=pulsos_contados_PUN_x, y=pulsos_contados_PUN_y)
    pw3.setXRange(set_point_PUN_x[0], set_point_PUN_x[window_size_j3-1])
    pw3.setYRange(-100, -40)
    if len(data_J1y_contados) > 2 and len(data_J2y_contados) > 2 and len(data_J3y_contados) > 2: 
        indice1 = len(data_J1y_contados) - 1
        indice2 = len(data_J2y_contados) - 1
        indice3 = len(data_J3y_contados) - 1
        

        teste,teste2= EulerOrientation(data_J1y_contados[indice1],data_J2y_contados[indice2],data_J3y_contados[indice3])
        data_contados_tx.append(teste[0])
        data_contados_ty.append(teste[1])
        


    #Centro da trajetória [x0 ;y0];
    centro = [0.36, 0.18]

    #Raio da trajetória [rx ;ry];
    raio = [0.10, 0.10]

    #Tempo total da trajetória, em segundos.
    #t_max = 20
    
    #Quantas voltas vai dar, dentro do tempo máximo.
    #voltas = 2

    t =np.linspace(0,1,500)
    loop_time = 1    
    pd = [centro[0] + raio[0]*np.cos(2*math.pi*t*loop_time),
          centro[1] + raio[1]*np.sin(2*math.pi*t*loop_time)]
    J4_setpoint.setData(x=pd[0], y=pd[1])
    J4_data.setData(x=data_contados_tx, y=data_contados_ty)
    
    np.savetxt("/home/jonathan/data_contados_tx.csv", data_contados_tx, delimiter=",")
    np.savetxt("/home/jonathan/data_contados_ty.csv", data_contados_ty, delimiter=",")
    pw4.setXRange(0.21, 0.49)
    pw4.setYRange(0.06, 0.34)


        

#Timer para atualizar o gráfico.
t = QtCore.QTimer()
t.timeout.connect(updateData)
t.start(50)

#Inicia o nó do ROS.
rospy.init_node('Supervisorio')
now_start = rospy.Time.now()
rospy.Subscriber("status_COT", status_arm, callback_COT)
rospy.Subscriber("status_OMB", status_arm, callback_OMB)
rospy.Subscriber("status_PUN", status_arm, callback_PUN)

#Inicia a GUI.
QtGui.QApplication.instance().exec_()

#Mantém o programa enquanto há conexão com ROS.
rospy.spin()
