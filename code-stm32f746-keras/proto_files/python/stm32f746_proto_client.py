import mnist_predict_pb2
import numpy as np
import serial
import time


class ProtoClient:
    def __init__(self, port = '/dev/ttyUSB0'):
        self._port = port
        self._ser = serial.Serial (
                        port='/dev/ttyUSB0',
                        baudrate=115200,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS
                    )
        self._ser.flushInput()
        self._ser.flushOutput()
        self._ser.isOpen()

    def __del__(self):
        self._ser.close()

    def read_version(self):
        print("Requesting version...")
        # Create message
        msg = mnist_predict_pb2.pb_msg_version()
        msg.version = 0
        # Send data
        self._ser.write(msg.SerializeToString())
        # Receive data
        time.sleep(1)
        data_raw = self._ser.read_all()
        msg.ParseFromString(data_raw)
        print("Version: %d" % msg.version)
        return msg.version

    def send_digit(self, fname='./digit.txt'):
        # Read digit
        digit = np.loadtxt(fname)
        # Reshape
        x = digit.reshape(28,28)
        x = np.expand_dims(x, axis=0)
        x = np.expand_dims(x, axis=3)
        x.shape
        msg = mnist_predict_pb2.pb_msg_predict_req()
        msg.buffer = x.tobytes()
        # Send digit data
        print('Send digit data to stm32...')
        self._ser.write(msg.SerializeToString())
        # Receive response
        time.sleep(3)
        data_raw = self._ser.read_all()
        # parse resp
        # resp = mnist_predict_pb2.pb_msg_predict_resp()
        # resp.ParseFromString(data_raw)
        print(data_raw)


if __name__=="__main__":

    cl = ProtoClient('/dev/ttyUSB0')
    # ver = cl.read_version()
    # if ver == 100:
    #     cl.send_digit()
    cl.send_digit()
