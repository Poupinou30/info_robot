#create a radial plot of the lidar exported data
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
data= np.genfromtxt("data.txt", delimiter=",")
angle=data[:, 0]
distance=data[:, 1]



#r = distance
#theta = 2 * np.pi * r

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.scatter(angle * (np.pi/180) , distance)
ax.set_rmax(6)
ax.set_rticks([0.5, 1, 2,3,4,5,6])  # Less radial ticks
ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
ax.grid(True)

ax.set_title("A line plot on a polar axis", va='bottom')
plt.show()
