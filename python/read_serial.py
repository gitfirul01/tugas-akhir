import serial, struct
 
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()
 
while True:
    # Reading all bytes available bytes till EOL
    data = ser.read(24) #.decode('utf-8').rstrip()
    value1 = struct.unpack('d', data[0:8])[0]
    value2 = struct.unpack('d', data[8:16])[0]
    value3 = struct.unpack('d', data[16:24])[0]
    print(value1)
    print(value2)
    print(value3)

    # Converting Byte Strings into unicode strings
    # string = line.decode()  
    # print(string)
    # Converting Unicode String into integer
    # a, b, c = string.split(';')
    # a = int(a)
    # b = int(b)
    # c = int(c)

    # print(a)
    # print(b)
    # print(c)
