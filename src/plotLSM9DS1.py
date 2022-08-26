from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import math 

# accel = np.dtype([('elapsedTime', float), ('sensorID', int), ('ax', float), ('ay', float), ('az', float), ('gx', float), ('gy', float), ('gz', float), ('mx', float), ('my', float), ('mz', float)])
# #accel=np.dtype([('f1', np.int16)])

DECLINATION = -11.583#// http://www.ngdc.noaa.gov/geomag-web/#declination

fname = '/home/novikov/PUEO/accelerometers/accel1.log'
data = np.genfromtxt(fname, skip_header=0, max_rows = 700000, dtype=None, encoding=None)

def getPitch(ax, ay, az):
    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) /math.pi*180.0
    return pitch

def getRoll(ay, az):
    roll = math.atan2(ay, az) /math.pi*180.0
    return roll

def getHeading(mx, my):
    heading = math.atan2(mx, my)*(-1)*DECLINATION*math.pi/180.0
    # if heading > math.pi:
    #     heading = heading - 2*math.pi
    # if heading < -math.pi:
    #     heading = heading + 2*math.pi
    return heading*180.0/math.pi

elapsedTime0 = []
elapsedTime1 = []
ax0 = []
ay0 = []
az0 = []
gx0 = []
gy0 = []
gz0 = []
mx0 = []
my0 = []
mz0 = []
ax1 = []
ay1 = []
az1 = []
gx1 = []
gy1 = []
gz1 = []
mx1 = []
my1 = []
mz1 = []
pitch0 = []
roll0 = []
heading0 = []
pitch1 = []
roll1 = []
heading1 = []
g0 = []
g1 = []

def parseData():
    for i in range(len(data)):
        if data[i][1] == 0: #sensor0
            elapsedTime0.append(data[i][0])
            ax0.append(data[i][2])
            ay0.append(data[i][3])
            az0.append(data[i][4])
            gx0.append(data[i][5])
            gy0.append(data[i][6])
            gz0.append(data[i][7])
            mx0.append(data[i][8])
            my0.append(data[i][9])
            mz0.append(data[i][10])
            pitch0.append(getPitch(data[i][2], data[i][3], data[i][4]))
            roll0.append(getRoll(data[i][3], data[i][4]))
            heading0.append(getHeading(data[i][8], data[i][9]))
            g0.append(math.sqrt(data[i][2]*data[i][2] + data[i][3]*data[i][3]+data[i][4]*data[i][4]))
        if data[i][1] == 1: #sensor0
            elapsedTime1.append(data[i][0])
            ax1.append(data[i][2])
            ay1.append(data[i][3])
            az1.append(data[i][4])
            gx1.append(data[i][5])
            gy1.append(data[i][6])
            gz1.append(data[i][7])
            mx1.append(data[i][8])
            my1.append(data[i][9])
            mz1.append(data[i][10])
            pitch1.append(getPitch(data[i][2], data[i][3], data[i][4]))
            roll1.append(getRoll(data[i][3], data[i][4]))
            heading1.append(getHeading(data[i][8], data[i][9]))
            g1.append(math.sqrt(data[i][2]*data[i][2] + data[i][3]*data[i][3]+data[i][4]*data[i][4]))

parseData()
fig, ax =  plt.subplots()
ax.plot(elapsedTime1, g0, color="blue", label = "g0")
ax.plot(elapsedTime1, g1, color="red", label = "g1")
ax.set_xlabel("Elapsed time [s]")
ax.set_ylabel("g (sqrt(ax^2+ay^2+az^2)) [g]")
ax.grid()
# ax.plot(elapsedTime1, gz1, color="green")
# plt.plot(elapsedTime1,ax1)
plt.show()
