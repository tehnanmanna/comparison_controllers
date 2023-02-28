#!/usr/bin/env python3


import pandas as pd
import matplotlib.pyplot as plt
import csv

# list of CSV files to read
files = ['/home/tehnanmanna/comparison_nav_ws/src/script_paper/script_paper/graphs/simple_world/DWB/cpu_memory_usage_for_DWB.csv', '/home/tehnanmanna/comparison_nav_ws/src/script_paper/script_paper/graphs/simple_world/RPP/cpu_memory_usage_for_RPP.csv', '/home/tehnanmanna/comparison_nav_ws/src/script_paper/script_paper/graphs/simple_world/TEB/cpu_memory_usage_for_TEB.csv']

# list of colors to use for each dataset
colors = ['red', 'blue', 'green']

# initialize a figure object
fig, ax = plt.subplots()

# loop through each CSV file and plot the data
for i, file in enumerate(files):
    # read the CSV file into a pandas dataframe
    df = pd.read_csv(file)

    # extract the x and y data
    # x = df['Time']
    y = df['Memory Usage (MB)']

    # plot the data using a line graph with the corresponding label name and color
    if i == 0:
        ax.plot(y, color=colors[i], label='DWB')
    elif i == 1:
        ax.plot(y, color=colors[i], label='RPP')
    elif i == 2:
        ax.plot(y, color=colors[i], label='TEB')

# add a legend to the graph
ax.legend()

# set the x and y axis labels
ax.set_xlabel('Time')
ax.set_ylabel('Memory Usage (MB)')

# set the title of the graph
ax.set_title('Memory Usage (MB) over Time for a simple straight goal in a simulation')

# show the graph
plt.show()
