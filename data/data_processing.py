# -*- coding: utf-8 -*-
"""
Created on Sun Jan 10 21:54:17 2021

@author: Simon van Gemert
"""
import matplotlib
import matplotlib.pyplot as plt
import numpy as np


def plotEenBar(data1, data2, what):
    labels = ['G1', 'G2', 'G3', 'G4', 'G5']
    
    x = np.arange(5)  # the label locations
    width = 0.35  # the width of the bars
    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width/2, data1, width, label='funnel')
    rects2 = ax.bar(x + width/2, data2, width, label='no-funnel')
    
    ax.set_ylabel(what)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    
    def autolabel(rects):
         """Attach a text label above each bar in *rects*, displaying its height."""
         for rect in rects:
             height = rect.get_height()
             ax.annotate('{}'.format(np.round(height,decimals=3)),
                         xy=(rect.get_x() + rect.get_width() / 2, height),
                         xytext=(0, 3),  # 3 points vertical offset
                         textcoords="offset points",
                         ha='center', va='bottom')

    autolabel(rects1)
    autolabel(rects2)
    
    fig.tight_layout()
    
    plt.show()

timeFunnel = [6.568, 11.957, 50.033, 86.792, 61.45]
timeNone = [16.112, 21.477, 58.043, 90.307, 89.178]
print(timeFunnel)

data_funnel = []
for i in range(20):
    name = "funnel/data"+str(i) + ".csv"
    data_funnel.append(np.genfromtxt(name, delimiter=","))
avg_funnel = np.mean(data_funnel, axis=0)
print("funnel: ", avg_funnel)

data_non = []
for i in range(10):
    name = "non/data"+str(i) + ".csv"
    data_non.append(np.genfromtxt(name, delimiter=","))
avg_non = np.mean(data_non, axis=0)
print("no_funnel: ", avg_non)

#flight
plotEenBar(timeFunnel, timeNone, 'Flight time[s]')
#times
plotEenBar(avg_funnel[:,0], avg_non[:,0], 'Time[s]')
#nodes
plotEenBar(avg_funnel[:,1], avg_non[:,2], 'Node')
#iters
plotEenBar(avg_funnel[:,2], avg_non[:,2], 'Iterations')




