import matplotlib as plt
from mpl_toolkits import mplot3d
import csv

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

x = []
y = []
z = []
x2 = []
y2 = []
z2 = []
x3 = []
y3 = []
z3 = []
x4 = []
y4 = []
z4 = []
x5 = []
y5 = []
z5 = []
count = 0

with open('oscillate_vision.csv', 'r') as file:
    reader = csv.reader(file)
    for row in reader:
        count+=1
        print(row)
        c = 0
        for r in row:
            c+=1
            r = float(r)
            # if count < 6308:
            #     if c == 1:
            #         x.append(r)
            #     if c == 2:
            #         y.append(r)
            #     if c == 3:
            #         z.append(r)
            # else:
            #     if c == 1:
            #         x2.append(r)
            #     if c == 2:
            #         y2.append(r)
            #     if c == 3:
            #         z2.append(r)
            # if count<10827:
            #     if c == 7:
            #         x3.append(r)
            #     if c == 8:
            #         y3.append(r)
            #     if c == 9:
            #         z3.append(r)
            # else:
            #     if c == 7:
            #         x4.append(r)
            #     if c == 8:
            #         y4.append(r)
            #     if c == 9:
            #         z4.append(r)
            # if c == 13:
            #     x5.append(r)
            # if c == 14:
            #     y5.append(r)
            # if c == 15:
            #     z5.append(r)


            if c == 1:
                x.append(r)
            if c == 2:
                y.append(r)
            if c == 3:
                z.append(r)
            if c == 7:
                x2.append(r+x[len(x)-1])
            if c == 8:
                y2.append(r+y[len(y)-1])
            if c == 9:
                z2.append(0.4358)
            if c == 13:
                x3.append(r)
            if c == 14:
                y3.append(r)
            if c == 15:
                z3.append(0.43582)


            # a = ''
            # # print(r)
            # c = 0
            # for p in r:
            #     # print(p)
            #     if p !=';':
            #         a = a+p
            #     else:
            #         c+=1
            #         a = float(a)
            #         if c == 1:
            #             x.append(a)
            #         if c == 2:
            #             y.append(a)
            #         if c == 3:
            #             z.append(a)
            #         a = ''
            #for p in r:
                #print(p)
        #x.append(row[0][0])
        #y.append(row[0][1])
        #z.append(row[0][2])

# print(x,y,z)

fig = plt.figure()
ax = fig.gca(projection='3d')
ax = plt.axes(projection="3d")
ax.set_xlim([-10,10])
ax.set_ylim([-5,10])
ax.grid(True)
ax.plot(x, y, z, '-', color='blue', label='drone trajectory')
ax.plot(x2, y2, z2, '--', color='red', label='estimated rover trajectory')
ax.plot(x3, y3, z3, '-', color='green', label='actual rover trajectory')
# ax.plot(x2, y2, z2, '--', color='blue', label='drone1 trajectory after landing')
# ax.plot(x3, y3, z3, '-', color='green', label='drone2 trajectory')
# ax.plot(x4, y4, z4, '--', color='green', label='drone2 trajectory after landing')
# ax.plot(x5, y5, z5, color='red', label='target trajectory')
# ax.plot(x4, y4, z4, label='actual target trajectory')
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
ax.legend()
plt.show()
