import numpy as np

# def readLSM(file_name):
#     with open(file_name, 'r') as data:
#         elapsedTime = []
#         ax = np.array()
#         ay = np.array()
#         az = []
#         gx = []
#         gy = []
#         gz = []
#         mx = []
#         my = []
#         mz = []
#         for line in data:
#             p = line.split()
#             elapsedTime.append(float(p[0]))
#             ax.append(float(p[1]))
#             ay.append(float(p[2]))
#             az.append(float(p[3]))
#             gx.append(float(p[4]))
#             gy.append(float(p[5]))
#             gz.append(float(p[6]))
#             mx.append(float(p[7]))
#             my.append(float(p[8]))
#             mz.append(float(p[9]))

#     print(ax)
#     return elapsedTime, ax, ay, az, gx, gy, gz, mx, my, mz

#elapsedTime, ax, ay, az, gx, gy, gz, mx, my, mz = readLSM('/home/novikov/PUEO/accelerometers/accel.log')
# readLSM('/home/novikov/PUEO/accelerometers/accel.log')
#print(ax*ay) 

accel = np.dtype([('elapsedTime', float), ('sensorID', int), ('ax', float), ('ay', float), ('az', float), ('gx', float), ('gy', float), ('gz', float), ('mx', float), ('my', float), ('mz', float)])
#accel=np.dtype([('f1', np.int16)])
fname = '/home/novikov/PUEO/accelerometers/accel.log'
print(accel)
data = np.zeros((10,), dtype=accel)
data[3]['ax'] = 0.34
print(data)
print(np.fromfile(fname, dtype=accel, count= 10, sep=' '))
