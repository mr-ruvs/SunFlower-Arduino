import time
from Adafruit_IO import Client, RequestError
import serial

# Configura tu clave de Adafruit IO y el nombre de los feeds
ADAFRUIT_IO_KEY = 'aio_ctGS98322VCf0I9wcgQ72qSNo8no'
ADAFRUIT_IO_USERNAME = 'mrruvs'
FEED_IDS = ['servoh', 'servov', 'servoindicadorluz', 'derecha', 'izquierda', 'pwmdcmotor']

# Inicializa el cliente de Adafruit IO
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Configura el puerto serial (ajusta 'COM4' a tu puerto serial)
ser = serial.Serial('COM4', 9600, timeout=1)
time.sleep(2)  # Espera a que se establezca la conexión serial

last_values = [None] * len(FEED_IDS)

def read_adafruit_io(feed_id):
    try:
        data = aio.receive(feed_id)
        value = int(data.value)
        return value
    except RequestError as e:
        print(f'Error: {e}')
        return None

while True:
    for i, feed_id in enumerate(FEED_IDS):
        value = read_adafruit_io(feed_id)
        if value is not None and value != last_values[i]:
            print(f'Received new value from {feed_id}: {value}')
            # Envía los datos como una cadena de 3 bytes
            data_to_send = f'{value:03}'  # Asegura que el valor tenga 3 dígitos
            ser.write(data_to_send.encode('utf-8'))
            last_values[i] = value
            
