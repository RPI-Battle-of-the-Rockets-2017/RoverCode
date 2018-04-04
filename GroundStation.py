import serial
import time
from cobs import cobs
import binascii
import struct
import argparse
import io
import time

ser = serial.Serial('COM17', 9600)
msg = ''

args_parser = argparse.ArgumentParser(description='Ground station')
args_parser.add_argument("--save_all_data", help="Saves all data to a file", action="store_true")
args_parser.add_argument("--save_baro_data", help="Saves the barometer data to a file", action="store_true")
args = args_parser.parse_args()

f = None
image_num = 0

baro_file = None
start_time = time.time()
if args.save_baro_data:
    baro_file = open('baro_data.txt', 'a', 0)

def ProcessMessage(msg):
    decoded = cobs.decode(msg)

    if len(decoded) >= 4:
        header = decoded[0]
        length = struct.unpack("<B", decoded[1])[0]
        sequence_num = decoded[2]
        msg_type = decoded[3]

        print(binascii.hexlify(decoded[0]))
        #print(binascii.hexlify(decoded[1]))
        print(length)
        print(binascii.hexlify(decoded[2]))
        print(binascii.hexlify(decoded[3]))

        if msg_type == '\x00':
            #ESTABLISH_COMM
            print("Establish comm")
            ser.write(cobs.encode(sequence_num) + '\x00')

        elif msg_type == '\x01':
            print("Initiate image")
            f = open('test.txt', 'w')
            ser.write(cobs.encode(sequence_num) + '\x00')

        elif msg_type == '\x02':
            print("Sending image")
            print(bytearray(decoded[4:]))
            f.write(bytearray(decoded[4:]))
            print(sequence_num + struct.pack('<B', length - 4))
            ser.write(cobs.encode(sequence_num + struct.pack('<B', length - 4)) + '\x00')

        elif msg_type == '\x03':
            print("End image")
            f.close();
            ser.write(cobs.encode(sequence_num) + '\x00')

        elif msg_type == '\x04': #BARO
            print("Baro")
            val = struct.unpack("<H", decoded[4:])[0]
            print(val)
            if (args.save_baro_data):
                msg = str(time.time() - start_time) + "," + str(val) + "\n"
                baro_file.write(msg)
            ser.write(cobs.encode(sequence_num) + '\x00')

        elif msg_type == '\x05': #GPS
            print("GPS")
            ser.write(cobs.encode(sequence_num) + '\x00')

ser.reset_input_buffer()
while 1:
    if ser.inWaiting():
        newByte = ser.read()
        if newByte == '\x00':
            try:
                ProcessMessage(msg)
            except:
                pass
            msg = ''
        else:
            msg += newByte
