import serial, struct
 
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
 
while True:
    # Reading all bytes available bytes till EOL
    # data = ser.read(12)
    # value1 = struct.unpack('i', data[8:12])[0]
    # value2 = struct.unpack('i', data[4:8])[0]
    # value3 = struct.unpack('i', data[0:4])[0]
    # print(value1)
    # print(value2)
    # print(value3)

    if ser.read(1) == b'@':
        data = ser.read_until(b'$')
        data = data[1:len(data)-1]
        try:
            value = struct.unpack('fff', data)
            print(value)
        except:
            None

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
