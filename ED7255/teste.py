#Importando as bibliotecas:
import ed7255 as ed

#Declaração do objeto do robô:
robo = ed.ED7255()

#Iniciando a amostragem:
robo.initSampling()

#Movimentação livre joint:
robo.moveJoint([30, -10, -20, -30, 50])
robo.moveJoint([-20, -30, -80, -15, 45])

#Movimentação livre cartesiana:
robo.move([300, 300, 200, 90, 0])
robo.move([584.5, 0, 187, 90, 0])

#Movimento linear:
robo.moveLinear([500, 0, 187, 90, 0])
robo.moveLinear([460, 0, 187, 90, 0])

#Movimento incremental via base-jog:
robo.baseJog([0, 0, 13, -30, 0])

robo.toolJog([0, 0, -200, 0, 0])

#Plotando as curvas:
robo.plotData()