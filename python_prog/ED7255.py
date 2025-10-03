#importando as bibliotecas necessárias:
import numpy as np
import paho.mqtt.client as mqtt
import time

#Classe para enviar comandos à controladora:
class ED7255:
    #Contrutor para inicialização da classe e da conexão com a controladora:
    def __init__(self):
        #Configurando o cliente e conectando via MQTT:
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 'PCPY')
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect('192.168.137.1', 1884)
        self.client.loop_start()
        time.sleep(1)
        #Variáveis globais:
        self.moving = False
    
    #Função de callback executada quando a conexão for estabelecida:
    def on_connect(self, client, userdata, flags, reason_code, properties):
        #Mensagem de status da conexão:
        print(f'Connected with result code {reason_code}')
        #Inscrevendo nos tópicos:
        self.client.subscribe('reply')
        

    #Função de callback executada ao receber uma mensagem:
    def on_message(self, client, userdata, msg):
        #print(msg.topic+" "+msg.payload.decode('utf-8'))
        #Atribuição do valor recebido à variável correspondente:
        if(msg.topic == 'reply'):
            if(msg.payload.decode('utf-8') == 'X'):
                self.moving = False
                print('\n\n! Posição fora do espaço de trabalho do robô !')
            if(msg.payload.decode('utf-8')[0] == 'W'):
                #self.moving = False
                dof = int(msg.payload.decode('utf-8')[1])
                print(f'\n\n! Limite de movimentação alcançado na Junta {dof} !')
            if(msg.payload.decode('utf-8')[1] == '1'):
                self.moving = False
                print(f'\n\n* Pose final alcançada *')



    #Funções gerais da controladora:
    def start(self):
        self.client.publish('request', 'S')
    def hold(self):
        self.client.publish('request', 'H')
    def setSPDOVR(self, SPDOVR):
        self.client.publish('request', f'D{SPDOVR},')

    #Funções para movimentação:
    def moveJoint(self, angles):
        self.client.publish('request', f'J{angles[0]},{angles[1]},{angles[2]},{angles[3]},{angles[4]},')
        self.moving = True
    def moveLinear(self, pose):
        self.client.publish('request', f'L{pose[0]},{pose[1]},{pose[2]},{pose[3]},{pose[4]},')
        self.moving = True
    def moveAbout(self, x, y, z, intool=False):
        if(intool):
            self.client.publish('request', f'B{x},{y},{z},')
        else:
            self.client.publish('request', f'T{x},{y},{z},')
        self.moving = True
    
    #Função para a garra:
    def openGrip(self, opening=80):
        self.client.publish('request', f'G6,{opening},')
    def closeGrip(self, opening=50):
        self.client.publish('request', f'G6,{opening},')

    #Função para aguardar a movimentação:
    def waitMove(self):
        while(True):
            time.sleep(10e-3)
            if(not self.moving):
                break