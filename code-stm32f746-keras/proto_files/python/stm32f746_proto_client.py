import mnist_predict_pb2
import numpy as np
import serial
import time

req = mnist_predict_pb2.MnistDigitPredictRequest()

ser = serial.Serial (
    port='/dev/ttyUSB1',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

# Get digit
digit = np.loadtxt('digit.txt')
# Reshape
x = digit.reshape(28,28)
x = np.expand_dims(x, axis=0)
x = np.expand_dims(x, axis=3)
x.shape

req.buffer = x.tobytes()

ser.write(req.SerializeToString())

time.sleep(1)
out = ser.read_all()
    
if out != '':
    print(out)