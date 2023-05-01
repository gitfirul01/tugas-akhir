import serial, struct

## confgure serial communication
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
except:
    print("Serial not connected!")
    exit()


while True:
    ## read joint angle from serial
    if ser.read(1) == b'@':
        data = ser.read_until(b'$')
        data = data[1:len(data)-1]
        try:
            value = struct.unpack('fff', data)
            theta1 = value[0]
            theta2 = value[1]
            theta3 = value[2]
        except:
            pass