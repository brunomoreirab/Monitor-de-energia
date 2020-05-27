import dash
from dash.dependencies import Output, Input
import dash_core_components as dcc
import dash_html_components as html
import plotly
import plotly.graph_objs as go
from collections import deque
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import logging
import time
import json
import sqlite3
from datetime import datetime
import os
from urllib.request import pathname2url

now = datetime.now()
last_month = now.month
energy = 0

# Verifica se ja existia um banco de dados
try:
    dburi = 'file:{}?mode=rw'.format(pathname2url("example.db"))
    conn = sqlite3.connect(dburi, uri=True, check_same_thread=False)
    c = conn.cursor()
    c.execute('''DROP TABLE monitor''')
except sqlite3.OperationalError:
    conn = sqlite3.connect('example.db', check_same_thread=False)
    c = conn.cursor()

# Cria a tabela
c.execute('''CREATE TABLE monitor 
            (I_PHA1, P_PHA1, I_PHA2, P_PHA2, I_CIR1, P_CIR1, I_CIR2, P_CIR2, I_CIR3, P_CIR3, energy, date)''')
conn.commit()
c.close()

# Callback MQTT
def mqttCallback(client, userdata, message):
    msg_json = json.loads(message.payload.decode('utf-8'))
    saveJSON(msg_json)
    print(message.payload.decode('utf-8'))

def isNeg(measurement):
    if measurement < 0:
        measurement = 0
    return measurement

# Salva a mensagem recebida no banco de dados
def saveJSON(msg_json):
    global last_month, energy

    ID = msg_json['ID']
    now = datetime.now()
    formatted_date = now.strftime('%Y-%m-%d %H:%M:%S')

    # Verifica se o mes continua o mesmo desde a ultima leitura e calcula a energia gasta no mes
    if now.month == last_month:
        energy += (msg_json['Phase 1']['POWER'] + msg_json['Phase 2']['POWER'])/(360*1000)
    else:
        energy = (msg_json['Phase 1']['POWER'] + msg_json['Phase 2']['POWER'])/(360*1000)

    # Atualiza o mes da leitura
    last_month = now.month

    # Insere os valores lidos no banco de dados
    c = conn.cursor()
    c.execute('''INSERT INTO monitor VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''',(
                                            isNeg(msg_json['Phase 1']['CURRENT']),
                                            isNeg(msg_json['Phase 1']['POWER']),
                                            isNeg(msg_json['Phase 2']['CURRENT']),
                                            isNeg(msg_json['Phase 2']['POWER']),
                                            isNeg(msg_json['Circuit 1']['CURRENT']),
                                            isNeg(msg_json['Circuit 1']['POWER']),
                                            isNeg(msg_json['Circuit 2']['CURRENT']),
                                            isNeg(msg_json['Circuit 2']['POWER']),
                                            isNeg(msg_json['Circuit 3']['CURRENT']),
                                            isNeg(msg_json['Circuit 3']['POWER']),
                                            energy,
                                            formatted_date))
    conn.commit()
    c.close()

# Configura a API da AWS IoT Core
host = "a10lekoccab51o-ats.iot.sa-east-1.amazonaws.com"
rootCAPath = "certs/root-CA.crt"
certificatePath = "certs/dash_python.cert.pem"
privateKeyPath = "certs/dash_python.private.key"
port = 8883
useWebsocket = False
clientId = "dash_python"
topic = "test_topic/esp32"
mode = "subscribe"

# Configura o logging
logger = logging.getLogger("AWSIoTPythonSDK.core")
logger.setLevel(logging.DEBUG)
streamHandler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
streamHandler.setFormatter(formatter)
logger.addHandler(streamHandler)

# Configuracao AWSIoTMQTTClient
myAWSIoTMQTTClient = AWSIoTMQTTClient(clientId)
myAWSIoTMQTTClient.configureEndpoint(host, port)
myAWSIoTMQTTClient.configureCredentials(rootCAPath, privateKeyPath, certificatePath)
myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)
myAWSIoTMQTTClient.configureDrainingFrequency(2)
myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)
myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)

# Conecta e subscreve ao topico do servidor
myAWSIoTMQTTClient.connect()
myAWSIoTMQTTClient.subscribe(topic, 1, mqttCallback)
time.sleep(2)

if __name__ == '__main__':
    while True:
        time.sleep(1)