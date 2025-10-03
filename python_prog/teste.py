#Biblioteca de funções para comandos do manipulador:
from ED7255 import *

#Objeto para a classe do manipulador:
robo = ED7255()

#Constantes e variáveis:
HOME = [0, 0, -5, 0, 0]
P1 = [90, -30, -60, 90, 0]
P2 = [-90, -60, -30, -90, 0]

#Comando para iniciar a movimentação do robô:
robo.start()

#Velocidade:
robo.setSPDOVR(50)

while True:
    #Ir para HOME:
    robo.moveJoint(HOME)
    robo.waitMove()

    #Ir para P1:
    robo.moveJoint(P1)
    robo.waitMove()

    #Ir para P2:
    robo.moveJoint(P2)
    robo.waitMove()