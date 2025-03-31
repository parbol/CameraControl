import numpy as np
import matplotlib.pyplot as plt
import sys

from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot
from src.cartesianpoint import cartesianpoint
from src.innerpoint import innerpoint


if __name__ == "__main__":


    fig = plt.figure(figsize = (16, 8), layout="constrained")
    gs0 = fig.add_gridspec(1, 2, width_ratios=[2, 1])
    ax1 = fig.add_subplot(gs0[0], projection = '3d')
    gs1 = gs0[1].subgridspec(2,1)
    ax2 = fig.add_subplot(gs1[0])
    ax3 = fig.add_subplot(gs1[1])
    ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.set_xlabel('x [cm]')
    ax1.set_ylabel('y [cm]')
    ax1.set_zlabel('z [cm]')
    ax2.set_xlabel('x [cm]')
    ax2.set_ylabel('y [cm]')
    ax3.set_xlabel('z [cm]')
    ax3.set_ylabel('y [cm]')
    ax1.axes.set_xlim3d(left=-70, right=70.0) 
    ax1.axes.set_ylim3d(bottom=-70, top=70.0)   
    ax2.axes.set_xlim((-40.0, 70.0))
    ax2.axes.set_ylim((-70.0, 40.0))
    ax3.axes.set_xlim((-1.0, 1.0))
    ax3.axes.set_ylim((-1.0, 1.0))


    table = Table(0.01, 0.0)
    table.plotTable(ax1, ax2, 'g.')

    # Generate the camera  
    camera = Camera(x = 5.0, y = 0.0, z = 2.0, psi = 0, theta = 0, phi = 0, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

    # Generate the robot
    robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)
    # 50.0, 30.0, 30.0, 40,

    # print('***TEST***')
    # v = [25, 20, 0]
    v = [-14.104129341237183, 19.38171754969979, 0.0]
    # v = [25,32,0]
    # v1 = [40,0,0]

    # robot.cartesianMoveTo(v,0)
    robot.cameraAim(v, 0)
    print('cameraPointing1', robot.cameraPointing())

    



    # PLOT/***********************************************
    # v = [20, 1,0]
    # table.plotTable(ax1, ax2, 'g.')
    # status, j1, j2, Z = robot.fromCartesianToInner(v)
    # jz = 0
    # pos = innerpoint( j1, j2, Z, jz)
    # ani = robot.animatedMove(pos, 10)
    # # print([np.sin(robot.currentPos.J1)*robot.R1, np.cos(robot.currentPos.J1)*robot.R1])
    # plt.show()
    # //***************************************************






    # robot.cameraAim(v, 0)

    # print('cameraPointing1', robot.cameraPointing())

 

    # v = [40,42,0]

    # robot.cartesianMoveTo(v, 0)

    # print('cameraPointing1', robot.cameraPointing())

    # robot.cartesianMoveTo(v, np.pi)

    # print('cameraPointing1', robot.cameraPointing())
    # robot.cameraAim(v, 0)

    # print('cameraPointing', robot.cameraPointing())
