import matplotlib
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import copy
import math
import socket 

# create and bind socket
localIP     = "127.0.0.1"
localPort   = 1100
bufferSize  = 1024
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind((localIP, localPort))

# Function adds received data from socket to visualisation
def update_vis(robot_pos_x,robot_pos_y,direction,battery_lvl,timeQI, pathQI):
    """# Apply trashes to plot if there are new trashes
    if ((trashes_coords[1] > 0) and ((trashes_coords[0] != previous_trashes_x[0]) or (trashes_coords[1] != previous_trashes_y[0]))):
        for k in range (-8,8,1):
            for j in range (-8,8,1):
                # Trashes - Blue 
                background[trashes_coords[0] + k,trashes_coords[1] + j,:] = [0,0,1]"""

    # Increase clean area and change robot position on plot if robot position has changed
    if (robot_pos_x != previous_robot_x[0]) or (robot_pos_y != previous_robot_y[0]):
        for k in [-2,-1,0,1,2,]:
            for j in [-2,0,-1,1,2]:
                if ((background[robot_pos_x + k,robot_pos_y + j,2] == 1)):
                    # Cleaned area - light green
                    background[robot_pos_x + k,robot_pos_y + j,:] = [210/255,1,210/255]
        # Robot path (center point) - Green
        background[robot_pos_x, robot_pos_y,:] = [0,0.99,0]

    # Update previous robot position and new trashes coords for next iteration
    previous_robot_x[0] = robot_pos_x
    previous_robot_y[0] = robot_pos_y
    # previous_trashes_x[0] = trashes_coords[0]
    # previous_trashes_y[0] = trashes_coords[1]

    i[0] = i[0] + 1
    # Display changes every 8th received packet
    if i[0] == 4:

        img[:,:,:] = background[:,:,:]
        i[0] = 0
        # Calculate direction and arrow properties
        direction = 2*3.1416*direction/360
        dx = 3*math.cos(direction)
        dy = -3*math.sin(direction)

        # Display robot position as circle-like area
        # Robot colour - Red
        img[robot_pos_x-2:robot_pos_x+3,robot_pos_y-2:robot_pos_y+3,:]=[1,0,0]
        img[robot_pos_x-3,robot_pos_y-1:robot_pos_y+2,:]=[1,0,0]
        img[robot_pos_x+3,robot_pos_y-1:robot_pos_y+2,:]=[1,0,0]
        img[robot_pos_x-1:robot_pos_x+2,robot_pos_y-3,:]=[1,0,0]
        img[robot_pos_x-1:robot_pos_x+2,robot_pos_y+3,:]=[1,0,0]

        # Display direction arrow, robot position and title with useful information
        a = plt.arrow(robot_pos_y,robot_pos_x,dx,dy,width=0.9)
        axim1.set_data(img)
        plt.title('Poziom baterii: '+ str(round(float(battery_lvl),2)) 
                + '%, timeQI: ' + str(round(float(timeQI),2))
                + 'sec, pathQI: ' + str(round(pathQI,2))
                + 'm,\n rotationQI: ' + str(round(rotation_qi,2)) 
                + 'deg, coverageQI: ' + str(round(coverage_qi,2)) + '%')
        # flush 
        fig1.canvas.flush_events()
        #fig1.canvas.draw()
        a.remove()
    return


# Initialize image for visualization and prepare data
matplotlib.pyplot.ion()
fig1, ax1 = plt.subplots()
background = mpimg.imread('../../roomba/plan/plan8.png')
img = copy.deepcopy(background)
axim1 = ax1.imshow(img)
plt.axis("off")
fig1.set_size_inches(8,8)


# Mutual objects help with handling first iteration and "did it change?" checking
i = [3]
previous_robot_x = [0]
previous_robot_y = [0]
# previous_trashes_x = [0]
# previous_trashes_y = [0]


# Main infinite loop waiting for data from socket
while(True):

    # Receive and unpack data 
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    clientMsg =  message
    clientIP  = "Client IP Address:{}".format(address)
    
    # Decode message
    clientMsg = clientMsg.decode("ISO-8859-1")
    # clientMsg = clientMsg.decode("utf-8")

    # Received variables:
    pos_x = ''
    pos_y = ''
    orientation = ''
    battery = ''
    timeQI = ''
    new_trashes_x = ''
    new_trashes_y = ''
    pathQI = ''
    rotation_qi = ''
    coverage_qi = ''

    # Each variable is separated by ' ' sign
    licznik_spacji = 0
    for char in clientMsg:
        if char == ' ':
              licznik_spacji += 1
        else:
            if licznik_spacji == 0:
                pos_x = pos_x + char
            elif licznik_spacji == 1:
                pos_y = pos_y + char
            elif licznik_spacji == 2:
                orientation = orientation + char
            elif licznik_spacji == 3:  
                battery = battery + char
            elif licznik_spacji == 4:
                timeQI = timeQI + char
            elif licznik_spacji == 5:
                pathQI = pathQI + char
            elif licznik_spacji == 6:
                rotation_qi = rotation_qi + char
            elif licznik_spacji == 7:
                coverage_qi = coverage_qi + char
            else:
                break

            """
            code for trashes coords
            elif licznik_spacji == 5:
                new_trashes_x = new_trashes_x + char
            elif licznik_spacji == 6:
                new_trashes_y = new_trashes_y + char
            """

    # Convert variable strings to actual values
    robot_pos_x = 400 - round(20*float(pos_y))
    robot_pos_y = round(20*float(pos_x))
    orientation = float(orientation)
    # trashes_x = 400 - round(2*float(new_trashes_x))
    # trashes_y = round(2*float(new_trashes_y))
    pathQI = float(pathQI)
    rotation_qi = float(rotation_qi)
    coverage_qi = 100*float(coverage_qi)
    #coverage_qi = 100*float(coverage_qi)
    # Call function to update display
    update_vis(robot_pos_x,robot_pos_y,orientation,battery,timeQI,pathQI)
