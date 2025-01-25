import numpy as np
import time
import paho.mqtt.client as mqtt

#Função de callback executada quando a conexão for estabelecida:
def on_connect(client, userdata, flags, reason_code, properties):
    #Mensagem de status da conexão:
    print(f'Connected with result code {reason_code}')
    #Inscrevendo nos tópicos:
    #client.subscribe('ESP32-COM', 0)

#Função de callback executada ao receber uma mensagem:
def on_message(client, userdata, msg):
    print(msg.topic+" "+msg.payload.decode('utf-8'))

#Inicializando a comunicação MQTT:
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 'MANAGER')
client.on_connect = on_connect
client.on_message = on_message
client.connect('192.168.137.1', 1884)

t0 = time.time()
while True:
    val = np.round(np.sin((time.time()-t0)*2)*100, 1)
    client.publish('sampling1', f'{val}, {val}, {val}, {val}, {val}, {val}')
    client.publish('sampling2', f'{val}, {val}, {val}, {val}, {val}, {val}')
    client.publish('sampling3', f'{val}, {val}, {val}, {val}, {val}, {val}')
    time.sleep(5*10e-3)