import socket
from time import sleep
import keyboard

# Function to send channel values to the server
def send_channel_values(client_socket, values):
    values_str = ','.join(map(str, values))
   
    client_socket.sendall(values_str.encode())
    print(values_str)

# Initialize variables for channel values
channel1 = 1500  # Center position
channel2 = 1500  # Center position
channel3 = 1500  # Center position
channel4 = 1500 
channel5 = 1500  # Center position

# Create a TCP/IP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the server wwwwsssssssadon Raspberry Pi
server_address = ('172.17.61.63',1243)
print('Connecting to {} port {}'.format(*server_address))
client_socket.connect(server_address)

# Main loop
running = True
while running:
    # Update channel values based on keyboard inputs
    if keyboard.is_pressed('a'):              
        channel1 = max(1000, channel1 - 40)  # Decrease channel 1 value
    elif keyboard.is_pressed('d'):
        channel1 = min(2000, channel1 + 40)  # Increase channel 1 value

    if keyboard.is_pressed('w'):              #for bldc
        channel2 = max(1000, channel2 - 40)  # Decrease channel 2 value
    elif keyboard.is_pressed('s'):
        channel2 = min(2000, channel2 + 40)  # Increase channel 2 value

    if keyboard.is_pressed('up'):             #for pitch
        channel3 = max(1000, channel3 - 40)  # Decrease channel 3 value
    elif keyboard.is_pressed('down'):
        channel3 = min(2000, channel3 + 40)  # Increase channel 3 value

    if keyboard.is_pressed('left'):           #For Roll
        channel4 = max(1000, channel4 - 40)  # Decrease channel 4 value
    elif keyboard.is_pressed('right'):
        channel4 = min(2000, channel4 + 40)  # Increase channel 4 value
    else:
        # If no key is pressed, return channel 4 value to the original position
        channel4 = 1500
    if keyboard.is_pressed('f'):              #camera
        channel5 = max(1000, channel5 - 40)  # Decrease channel 3 value
    elif keyboard.is_pressed('v'):
        channel5 = min(2000, channel5 + 40) 

    # Send channel values to the server
    send_channel_values(client_socket, [channel1, channel2, channel3, channel4,channel5])

    # Print channel values
    # print("Channel 1: {}, Channel 2: {}, Channel 3: {}, Channel 4: {},Channel 5: {} ".format(channel1, channel2, channel3, channel4,channel5))
    # running=input()
    # Optional delay to limit loop rate
    sleep(0.1)

# Close the client socket
client_socket.close()
