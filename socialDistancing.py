# import package serial
import serial
# import package numpy
import numpy as np

lostSync = 0
gotHeader = 0


def serial_config():
    """
    function to configure the pi or computer
    receive data and returns the serial port
    """
    # Open the serial ports for the configuration and the data ports

    # Raspberry pi
    # data_port = serial.Serial('/dev/ttyS0', 9600)

    # Windows
    data_port = serial.Serial('COM3', 9600)

    if data_port.isOpen():
        try:
            # throwing all the data stored at port coming from sensor
            data_port.flushInput()
        # if error is been thrown print it
        except Exception as err:
            print("Error " + str(err))
            data_port.close()
            exit()

    else:
        try:
            data_port.open()
        except Exception as err:
            print("Error " + str(err))
            data_port.close()
            exit()

    return data_port


def processData(data_port):
    '''
    processes the serial data
    :return: none
    '''
    data = 0
    global byteBuffer, byteBufferLength
    max_buffer_size = 2048
    last_byte = 127
    # if data available in serial port
    if data_port.in_waiting:
        read_buffer = data_port.read(data_port.in_waiting)
        print(read_buffer)
        byte_vec = np.frombuffer(read_buffer, dtype='uint8')
        byte_count = len(byte_vec)

        # Check that the buffer is not full, and then add the data to the buffer
        if (byteBufferLength + byte_count) < max_buffer_size:
            byteBuffer[byteBufferLength:byteBufferLength + byte_count] = byte_vec[:byte_count]
            byteBufferLength = byteBufferLength + byte_count

            # Check that the buffer has some data
            if byteBufferLength > 600:
                # check for all possible locations for 127
                possible_locs = np.where(byteBuffer == last_byte)[0]
                print(possible_locs)









# -------------------------    MAIN   -----------------------------------------
if __name__ == "__main__":
    # to fill the buffer with zeros and size is 2048 bytes
    byteBuffer = np.zeros(2 ** 11, dtype='uint8')
    # let buffer length be 0
    byteBufferLength = 0
    # Configure the serial port
    Data_port = serial_config()

    while True:
        try:
            # process serial data
            processData(Data_port)
        except KeyboardInterrupt:
            Data_port.close()
            break
