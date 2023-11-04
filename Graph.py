#!/usr/bin/env pybricks-micropython
import matplotlib.pyplot as plt
import numpy as np
import csv




class Graph:
       def __init__(self):
              pass
              
       def plot(self, speed, distance):
              
              x = np.arange(0,len(speed))  # Sample data.
              # my_list = list(zip(speed, distance))
              # sorted(my_list,key=lambda x: x[0])
              # print(my_list)
              # speed, distance = zip(*my_list)
              # # Note that even in the OO-style, we use `.pyplot.figure` to create the Figure.
              fig, ax = plt.subplots(figsize=(5, 2.7), layout='constrained')
              ax.plot(x, speed, label='speed')  # Plot some data on the axes.
              ax.plot(x, distance, label='distance')  # Plot some data on the axes.
              ax.set_xlabel('speed')  # Add an x-label to the axes.
              ax.set_ylabel('distance')  # Add a y-label to the axes.
              ax.set_title("Simple Plot")  # Add a title to the axes.
              ax.legend()  # Add a legend.
              # plt.style.use('_mpl-gallery')
              # # make data
              # #x = np.linspace(0,len(speed))
              # vitesse = speed
              # distance = vitesse
              # #On va faire distance par vitesse

              # # plot
              # fig, ax = plt.subplots()

              # #ax.plot(x, vitesse, linewidth=2.0)
              # ax.plot(vitesse, distance, linewidth=2.0)
       
       def show(self, speed, distance):
              fig, ax1 = plt.subplots(2, 1)

              # dt = 0.01

              # # Fixing random state for reproducibility
              # np.random.seed(19680801)


              # nse1 = np.random.randn(len(t))                 # white noise 1
              # nse2 = np.random.randn(len(t))                 # white noise 2
              # r = np.exp(-t / 0.05)

              # cnse1 = np.convolve(nse1, r, mode='same') * dt   # colored noise 1
              # cnse2 = np.convolve(nse2, r, mode='same') * dt   # colored noise 2

              # two signals with a coherent part and a random part
              # s1 = 0.01 * np.sin(2 * np.pi * 10 * t) + cnse1
              # s2 = 0.01 * np.sin(2 * np.pi * 10 * t) + cnse2
              
              time = 5000
              print (distance,speed)
              # ax1.plot(time, distance, label='Distance')
              # ax1.plot(time, speed, label='Vitesse')
              ax1[0].plot(time, distance, time, speed)
              ax1[0].set_xlabel('Time')
              ax1[0].set_ylabel('s1 and s2')
              plt.show()
       
       def plot3(self, speed, distance):
              x = np.arange(0, len(speed), 1)
              # y1 = 0.05 * x**2
              # y2 = -1 *y1
              x1 = np.stack((x,distance), axis = 1)
              fig, ax1 = plt.subplots()

              ax2 = ax1.twinx()
              ax1.plot(x, speed, 'g-')
              ax2.plot(x, distance, 'b-')


              ax1.set_xlabel('X data')
              ax1.set_ylabel('Y1 data', color='g')
              ax2.set_ylabel('Y2 data', color='b')

              plt.show()
       
       

       def plot4(self, speed, distance):

              # Génération de données fictives
              time = np.linspace(0, 10, len(speed))

              # Création du graphe
              fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

              # Sous-graphique pour la vitesse
              ax1.plot(time, speed, 'b')
              ax1.set_ylabel('Vitesse')

              # Sous-graphique pour la distance
              ax2.plot(time, distance, 'g')
              ax2.set_xlabel('Temps')
              ax2.set_ylabel('Distance')

              # Ajuster les marges pour éviter les superpositions
              plt.tight_layout()

              # Affichage du graphe
              plt.show()

              

my_graph = Graph()
with open('log_2023_05_05_20_12_05_472783.csv',mode = 'r' ,newline='') as csvfile:
       reader = csv.reader(csvfile)
       next(reader)
       my_list = list(reader)
       # col1_list, col2_list = zip(*my_list)
       col1_list = []
       col2_list = []
       for i in my_list:
              if type(i) == str:
                     pass
              else:
                     col1_list.append(i[0])
                     col2_list.append(i[1])
       print(col1_list)
       my_graph.plot4(col1_list,col2_list)
plt.show()