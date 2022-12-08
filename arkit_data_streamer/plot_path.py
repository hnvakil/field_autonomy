"""
Script to plot CSV generated from odometry recorder
"""

import matplotlib.pyplot as plt
import pandas as pd
import os.path

def main():
    # CSV needs to be in the same directory
    df = pd.read_csv(os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_path.csv"))
    # Plot odometry positions
    fig = plt.figure(figsize=(12, 12))
    ax = plt.axes(projection="3d")
    ax.scatter3D(df['x'], df['y'], df['z'])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_aspect('equal')
    plt.show()

if __name__ == '__main__':
    main()