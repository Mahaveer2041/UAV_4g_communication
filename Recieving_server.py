import socket
import serial
import time

# Function to receive channel values from the client
def receive_channel_values(server_socket):
    data = server_socket.recv(1024)
    return list(map(int, data.decode().split(',')))

# Function to send channel values to the Arduino
def send_channel_values(ser, values):
    
    ser.write(','.join(map(str, values)).encode() + b'\n')
    #ser.flush()  # Flush the serial buffer


# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('', 1243)  # Replace 1243 with your desired port number
print('Starting up on {} port {}'.format(*server_address))
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(1)

print('Waiting for a connection...')
connection, client_address = server_socket.accept()

# Establish serial connection with Arduino
arduino = serial.Serial('COM5', 115200)  # Replace '/dev/ttyUSB0' with your Arduino's serial port
#time.sleep(5)

try:
    print('Connection from', client_address)

    # Main loop
    while True:
        # Receive channel values from the laptop
        channel_values = receive_channel_values(connection)
        print("Got")

        # Print received channel valuesa
        #print("Received channel values:", channel_values)

        # Send channel values to Arduino
        send_channel_values(arduino, channel_values)
        print("Sent")
        #time.sleep(0.2)
        response = arduino.readline().decode('utf-8',"replace").rstrip()
        print("Arduino responded:", response)



finally:
    # Clean up the connection
    connection.close()
    arduino.close()  # Close the serial connection
