##############################################################
##############################################################
######################## Robot program #######################
##############################################################
##############################################################

import numpy as np
import matplotlib.pyplot as plt
import sys
import math as math
from matplotlib.animation import FuncAnimation, writers
from src.Camera import Camera
from src.EulerRotation import EulerRotation
from src.Table import Table
from src.innerpoint import innerpoint
from src.cartesianpoint import cartesianpoint

import logging
logger = logging.getLogger(__name__)

logging.basicConfig(format="{asctime} - {levelname} - {message}", style="{", datefmt="%Y-%m-%d %H:%M", level=logging.ERROR)  # level=logging.INFO o level=logging.ERROR



####################################################################
# Some definitions                                                 #
# The robot pointer has two systems of coordinates:                #
# Cartesian: (x, y, z) and JZ = position of the pointer and JZ     #
# Inner: (J1, J2, Z) and JZ = rotations of the axis                #
# The robot pointer position z is Z0 - Z                           #
####################################################################
# The robot has also two imporant systems of reference:            #
# The absolute one with respect to the table (x, y, z)             # 
# And the one associated to the second arm of the robot            # 
# with the center at the robot pointer                             #
####################################################################


class Robot:

    def __init__(self, h, R1, R2, Z0, table, camera, fig, ax1, ax2, ax3):

        #Robot parameters
        self.h = h
        self.R1 = R1
        self.R2 = R2
        #Z0 is the height of the pointer of the robot when Z = 0
        self.Z0 = Z0
        self.tol = 1e-8
        logging.info(f'Robot R1: {R1}, R2: {R2}, h: {h}, Z0: {Z0} tol: {self.tol}')
        #Camera and table
        self.camera = camera
        self.table = table
        #Current position in cartesian coordinates
        self.currentPos = innerpoint(0.0, 0.0, 33.0, 0.0)
        self.currentPosStart = innerpoint(0.0, 0.0, 33.0, 0.0)
        self.currentPosEnd = innerpoint(0.0, 0.0, 33.0, 0.0)
        self.currentCartesianPos = cartesianpoint(np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0]))
        self.jzrot = EulerRotation(0.0, 0.0, 0.0)
        #This is the definition of the field of the camera
        self.frame = [np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0]), np.asarray([0.0, 0.0, 0.0])]
        self.N = 0
        #Information for drawing
        self.fig = fig
        self.ax1 = ax1
        self.ax2 = ax2
        self.ax3 = ax3
        
        self.MoveRobotTo(self.currentPos)


    ######### Move the robot ###########################################
    def MoveRobotTo(self, pos):
        self.currentPos = pos
        self.jzrot.setFromAngles(pos.Jz, 0.0, 0.0)
        self.currentCartesianPos = self.fromInnerToCartesian(pos)
        # logging.info(f'Moving robot to J1: {pos.J1}, J2: {pos.J2}, Z: {pos.Z}, JZ: {pos.Jz}')
        # logging.info(f'Moving robot to x: {self.currentCartesianPos.r[0]}, y: {self.currentCartesianPos.r[1]}, z: {self.currentCartesianPos.r[2]}')
        #Update the position of the camera
        self.updateCameraGlobals()

    
    ######### Move the robot ###########################################
    def cartesianMoveTo(self, v, jz):
        
        status, j1, j2, Z = self.fromCartesianToInner(v)
        # print(status, j1, j2, Z )
        
        if status:
            pos = innerpoint(j1, j2, Z, jz)
            self.MoveRobotTo(pos)
        else:
            r = np.sqrt(v[0]**2 + v[1]**2)
            logging.error(f'There was an error moving the robot. R = {r}')
            sys.exit()     

    ######### Set camera globals #######################################
    def updateCameraGlobals(self):
        
        rcamera = np.asarray([self.currentCartesianPos.r[0], self.currentCartesianPos.r[1], self.currentCartesianPos.r[2]]) + self.camera.r0[0] * self.currentCartesianPos.ux + self.camera.r0[1] * self.currentCartesianPos.uy + (self.camera.r0[2]) * self.currentCartesianPos.uz
        raxis = np.asarray([self.currentCartesianPos.r[0], self.currentCartesianPos.r[1], self.currentCartesianPos.r[2]]) + (self.camera.r0[2]) * self.currentCartesianPos.uz
        diff = rcamera - raxis
        self.camera.cartesianpos.r = raxis + self.jzrot.apply(diff)
        uxcamera = self.jzrot.apply(self.currentCartesianPos.ux)
        uycamera = self.jzrot.apply(self.currentCartesianPos.uy)
        uzcamera = self.jzrot.apply(self.currentCartesianPos.uz)
        self.camera.cartesianpos.ux = self.camera.rotation0.apply(uxcamera)
        self.camera.cartesianpos.uy = self.camera.rotation0.apply(uycamera)
        self.camera.cartesianpos.uz = self.camera.rotation0.apply(uzcamera)
        # logging.info(f'Moving camera to x: {self.camera.cartesianpos.r[0]}, y: {self.camera.cartesianpos.r[1]}, z: {self.camera.cartesianpos.r[2]}')
        # logging.info(f'Camera ux vector: ({self.camera.cartesianpos.ux[0]}, {self.camera.cartesianpos.ux[1]}, {self.camera.cartesianpos.ux[2]})')
        # logging.info(f'Camera uy vector: ({self.camera.cartesianpos.uy[0]}, {self.camera.cartesianpos.uy[1]}, {self.camera.cartesianpos.uy[2]})')
        # logging.info(f'Camera uz vector: ({self.camera.cartesianpos.uz[0]}, {self.camera.cartesianpos.uz[1]}, {self.camera.cartesianpos.uz[2]})')

        
        p1 = [1.0, 1.0]
        p2 = [1.0, -1.0]
        p3 = [-1.0, -1.0]
        p4 = [-1.0, 1.0]
        self.frame[0] = self.cameraProjectionToPoint3D(p1)
        self.frame[1] = self.cameraProjectionToPoint3D(p2)
        self.frame[2] = self.cameraProjectionToPoint3D(p3)
        self.frame[3] = self.cameraProjectionToPoint3D(p4)


    ######### Set Check if a point is within the frame##################  
    def checkInFrame(self, p):
        
        x, y = self.point3DToCameraProjection(p)
        if x >= -1.0 and x <= 1.0 and y >= -1.0 and y <= 1.0:
            return True
        return False


    ########## Animated function ########################################
    def animation_function(self, i):
    
        a = math.floor(self.N/3.0)
        b = math.floor(2.0*self.N/3.0)
        j1 = 0
        j2 = 0
        z = 0
        if i <= a:
            j1 = self.currentPosStart.J1 + i * (self.currentPosEnd.J1 - self.currentPosStart.J1) / a
            j2 = self.currentPosStart.J2
            z = self.currentPosStart.Z
            jz = self.currentPosStart.Jz
            newpos = innerpoint(j1, j2, z, jz)
            self.MoveRobotTo(newpos)
            self.drawRobot(self.ax1, self.ax2, self.ax3, 'y')
        elif i > a and i <= b:
            k = i - a - 1
            j1 = self.currentPosEnd.J1
            j2 = self.currentPosStart.J2 + k * (self.currentPosEnd.J2 - self.currentPosStart.J2) / (b-a-1)
            z = self.currentPosStart.Z 
            jz = self.currentPosStart.Jz
            newpos = innerpoint(j1, j2, z, jz)
            self.MoveRobotTo(newpos)
            self.drawRobot(self.ax1, self.ax2, self.ax3, 'y')
        else:
            k = i - b - 1
            j1 = self.currentPosEnd.J1
            j2 = self.currentPosEnd.J2
            #z = self.currentPosStart.Z + k * (self.currentPosEnd.Z - self.currentPosStart.Z) / (self.N - 1 - b - 1)
            z = self.currentPosStart.Z
            jz = self.currentPosStart.Jz + k * (self.currentPosEnd.Jz - self.currentPosStart.Jz) / (self.N -1 - b -1)
            newpos = innerpoint(j1, j2, z, jz)
            self.MoveRobotTo(newpos)
            self.drawRobot(self.ax1, self.ax2, self.ax3, 'y')

    ########## Animated move ########################################
    def animatedMove(self, pos, N):
    
        self.currentPosStart = self.currentPos
        self.currentPosEnd = pos
        self.N = N
        ani = FuncAnimation(self.fig, self.animation_function, frames=N, interval=1.0, blit=False, repeat=False)
        return ani
   

    ######## Auxiliary function##########################################
    def angleFromSineCosine(self, s, c):

        if s >= 0:
            return np.arccos(c)
        else:
            return -np.arccos(c)


    #Auxiliary function to check whether two points are the same##########
    def checkValidConversion(self, v, j):

        x = self.R1 * np.cos(j[0]) + self.R2 * np.cos(j[1])
        y = self.R1 * np.sin(j[0]) + self.R2 * np.sin(j[1])
        if (x-v[0])**2 + (y-v[1])**2 < 1e-3:
            # print(f'true! \t J1: {j[0]:.2f} \t J2: {j[1] - j[0]:.2f}')
            return True
        # print('false!')
        return False


    ######################################################################
    def fromInnerToCartesian(self, pos):

        x = self.R1 * np.cos(pos.J1) + self.R2 * np.cos(pos.J2+pos.J1)
        y = self.R1 * np.sin(pos.J1) + self.R2 * np.sin(pos.J2+pos.J1)
        z = self.Z0 - pos.Z
        ux = np.asarray([np.cos(pos.J1+pos.J2), np.sin(pos.J1+pos.J2), 0.0])
        uy = np.asarray([-np.sin(pos.J1+pos.J2), np.cos(pos.J1+pos.J2), 0.0])
        uz = np.asarray([0.0, 0.0, 1.0])
        newpos = cartesianpoint(np.asarray([x, y, z]), ux, uy, uz)
        return newpos


    ######################################################################
    def fromCartesianToInner(self, v):

        x = v[0]
        y = v[1]
        z = v[2]
        Z = self.Z0 - z
        Delta = (x**2 + y**2 + self.R1**2 - self.R2**2)/(2.0*self.R1)
        a = (x**2 + y**2)
        b = -2.0 * Delta * x
        c = Delta**2 - y**2

        if b**2-4.0*a*c < 0.0:
            # print('fallo 1')
            return False, 0, 0, 0
    
        cosj1_p = (-b + np.sqrt(b**2-4.0*a*c))/(2.0*a)
        cosj2_p = (x - self.R1 * cosj1_p) / self.R2
        sinj1_pp = np.sqrt(1.0 - cosj1_p**2)
        sinj2_pp = (y - self.R1 * sinj1_pp) / self.R2
        sinj1_pm = -np.sqrt(1.0 - cosj1_p**2)
        sinj2_pm = (y - self.R1 * sinj1_pm) / self.R2

        cosj1_m = (-b - np.sqrt(b**2-4.0*a*c))/(2.0*a)
        cosj2_m = (x - self.R1 * cosj1_m) / self.R2
        sinj1_mp = np.sqrt(1.0 - cosj1_m**2)
        sinj2_mp = (y - self.R1 * sinj1_mp) / self.R2
        sinj1_mm = -np.sqrt(1.0 - cosj1_m**2)
        sinj2_mm = (y - self.R1 * sinj1_mm) / self.R2

        J1pp = self.angleFromSineCosine(sinj1_pp, cosj1_p)
        J1pm = self.angleFromSineCosine(sinj1_pm, cosj1_p)
        J1mp = self.angleFromSineCosine(sinj1_mp, cosj1_m)
        J1mm = self.angleFromSineCosine(sinj1_mm, cosj1_m)
        J2pp = self.angleFromSineCosine(sinj2_pp, cosj2_p)
        J2pm = self.angleFromSineCosine(sinj2_pm, cosj2_p)
        J2mp = self.angleFromSineCosine(sinj2_mp, cosj2_m)
        J2mm = self.angleFromSineCosine(sinj2_mm, cosj2_m)

        pairs = [[J1pp, J2pp], [J1pm, J2pm], [J1mp, J2mp], [J1mm, J2mm]]
        # print('-----------------------')
        index = -1

        j1_min = np.inf
        for i, j in enumerate(pairs):
            if self.checkValidConversion(v, j):
                # This I still need to think about it
                if j[0] < j1_min:
                    index = i
                    j2_min = j[0]


        if index == -1:
            # print('fallo 2')
            return False, 0, 0, 0
        else:
            return True, pairs[index][0], pairs[index][1]-pairs[index][0], Z

    #Projection of a point into the camera
    def point3DToCameraProjection(self, r):

        s = self.camera.cartesianpos.r - r
        l = self.camera.focaldistance / (s[0]*self.camera.cartesianpos.uz[0] + s[1]*self.camera.cartesianpos.uz[1] + s[2]*self.camera.cartesianpos.uz[2])
        p = self.camera.cartesianpos.r + l * (self.camera.cartesianpos.r - r)
        
        center = self.camera.cartesianpos.r + self.camera.focaldistance * self.camera.cartesianpos.uz
        p = p - center


        # print(p[0]*self.camera.cartesianpos.ux[0], p[1]*self.camera.cartesianpos.ux[1], p[2]*self.camera.cartesianpos.ux[2])
        # print(p[0]*self.camera.cartesianpos.uy[0], p[1]*self.camera.cartesianpos.uy[1], p[2]*self.camera.cartesianpos.uy[2])


        x = self.camera.cx * (p[0]*self.camera.cartesianpos.ux[0] + p[1]*self.camera.cartesianpos.ux[1] + p[2]*self.camera.cartesianpos.ux[2])
        y = self.camera.cy * (p[0]*self.camera.cartesianpos.uy[0] + p[1]*self.camera.cartesianpos.uy[1] + p[2]*self.camera.cartesianpos.uy[2])
        return x, y
    
    #3D reconstruction point from camera
    def cameraProjectionToPoint3D(self, p):
        
        x = p[0]/self.camera.cx
        y = p[1]/self.camera.cy  
       
        center = self.camera.cartesianpos.r + self.camera.focaldistance * self.camera.cartesianpos.uz

        t = x * self.camera.cartesianpos.ux + y * self.camera.cartesianpos.uy + center
        s = self.camera.cartesianpos.r - t
        
        l = (self.table.z-self.camera.cartesianpos.r[2])/s[2]
        point3D = self.camera.cartesianpos.r + l * s
        return point3D
    
    def cameraPointing(self): 
        # Returns intersection between camera z pointing and table
        # Saber a que punto apunta la camara
        z_table = self.table.z
        camera_r = self.camera.cartesianpos.r  
        pointing = self.camera.cartesianpos.uz

        t = (z_table - camera_r[2])/pointing[2]

        x = camera_r[0] + t*pointing[0]
        y = camera_r[1] + t*pointing[1]
        z = self.camera.cartesianpos.r[2] - self.camera.focusdistance

        return [x,y,z]

    def cameraAim_developing(self, point: list, jz: float = 0):
        # Apuntar  a un punto con la camara
        # Primero giro la camara al jz deseado y luego muevo el robot

        self.cartesianMoveTo(point, jz)

        pointing = self.camera.cartesianpos.uz

        # Position where we want to move the camera   --> Depende de Jz?
        camera_r = np.asarray([0, 0, 0]) 

        t = self.camera.focusdistance/(np.linalg.norm(pointing))

        # print('t:', t)
        # print('pointing:', pointing)

        camera_r = point + t *pointing
        # print('camera_r:', camera_r)
        # print('camera.cartesianpos.r:',self.camera.cartesianpos.r)
        camera_robot = self.currentCartesianPos.r -  self.camera.cartesianpos.r 
        # print('camera_robot:',np.linalg.norm(camera_robot[0:2]))

        # Posicion a la que mover el robot:
        r_objective_robot = camera_r + camera_robot
        # r_objective_robot_table = np.append( r_objective_robot[0:2])
        # print('r_objective_robot:', r_objective_robot)
        self.cartesianMoveTo(r_objective_robot, jz)
        # print('cartesianpos:', self.camera.cartesianpos.r)

        # print('camera_r:', camera_r)
        # print('camera.cartesianpos.r:',self.camera.cartesianpos.r)
        camera_robot = self.currentCartesianPos.r -  self.camera.cartesianpos.r 
        # print('camera_robot:',np.linalg.norm(camera_robot[0:2]))

        # Posicion a la que mover el robot:
        r_objective_robot = camera_r + camera_robot
        # r_objective_robot_table = np.append( r_objective_robot[0:2])
        # print('r_objective_robot:', r_objective_robot)
        self.cartesianMoveTo(r_objective_robot, jz)

    def cameraAim(self, cartesianpoint: list, jz: float = 0):

        point = np.array([cartesianpoint[0], cartesianpoint[1]])
        
        radius_cam_robot = np.linalg.norm(self.camera.r0[0:2])

        pos_robot = np.array([point[0] + np.cos(jz)*radius_cam_robot, 
                              point[1] + np.sin(jz)*radius_cam_robot,
                              self.camera.focusdistance])
        
        self.cartesianMoveTo(pos_robot, 0)

        if radius_cam_robot != 0:

            r_p = (point - pos_robot[0:2])/np.linalg.norm(point - pos_robot[0:2])
            # vector robotR2-point

            r_r1 = np.array([np.cos(self.currentPos.J1)*self.R1, np.sin(self.currentPos.J1)*self.R1])

            # vector robotR1
            r_r2 = pos_robot[0:2]

            r_0 = (r_r2 - r_r1)/np.linalg.norm(r_r2 - r_r1)
            # vector robotR1-robotR2

            theta = np.arccos(np.dot(r_0, r_p)/(np.linalg.norm(r_p)*np.linalg.norm(r_0)))

            var = r_0 * np.matrix([[0, 1],[-1, 0]]) * np.array([[r_p[0]], [r_p[1]]])
        
            if var >= 0:
                jz = -theta
            elif var < 0:
                jz = theta
            
            self.cartesianMoveTo(pos_robot, jz)




    # Drawing the robot
    def drawRobot(self, ax1, ax2, ax3, t, alpha=1.0):
    
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax1.axes.set_xlim3d(left=-70, right=70.0) 
        ax1.axes.set_ylim3d(bottom=-70, top=70.0)   
        ax2.axes.set_xlim((-40.0, 70.0))
        ax2.axes.set_ylim((-70.0, 40.0))
        ax3.axes.set_xlim((-1.0, 1.0))
        ax3.axes.set_ylim((-1.0, 1.0))
        self.table.plotTable(ax1, ax2, 'g.')
        p1 = [0, 0, 0]
        p2 = [0, 0, self.h]
        p3 = [self.R1 * np.cos(self.currentPos.J1), self.R1 * np.sin(self.currentPos.J1), self.h]
        p4 = [p3[0] + self.R2 * np.cos(self.currentPos.J1+self.currentPos.J2), p3[1] + self.R2 * np.sin(self.currentPos.J1+self.currentPos.J2), self.h]
        p5 = [p4[0], p4[1], self.Z0]
        p6 = [p4[0], p4[1], self.currentCartesianPos.r[2]]
        x_start = [p1[0], p2[0], p3[0], p4[0], p5[0]]
        y_start = [p1[1], p2[1], p3[1], p4[1], p5[1]]
        z_start = [p1[2], p2[2], p3[2], p4[2], p5[2]]
        x_start2 = [p5[0], p6[0]]
        y_start2 = [p5[1], p6[1]]
        z_start2 = [p5[2], p6[2]]
               
        p7 = np.asarray([p6[0], p6[1], self.camera.cartesianpos.r[2]])
        p8 = self.camera.cartesianpos.r
        x_start3 = [p7[0], p8[0]]
        y_start3 = [p7[1], p8[1]]
        z_start3 = [p7[2], p8[2]]

        k1 = [1.0, 1.0]
        k2 = [1.0, -1.0]
        k3 = [-1.0, -1.0]
        k4 = [-1.0, 1.0]
        k1p = self.cameraProjectionToPoint3D(k1)
        k2p = self.cameraProjectionToPoint3D(k2)
        k3p = self.cameraProjectionToPoint3D(k3)
        k4p = self.cameraProjectionToPoint3D(k4)

        k1p = self.frame[0]
        k2p = self.frame[1]
        k3p = self.frame[2]
        k4p = self.frame[3]

        x_start4 = [p8[0], k1p[0]]
        y_start4 = [p8[1], k1p[1]]
        z_start4 = [p8[2], k1p[2]]
        x_start5 = [p8[0], k2p[0]]
        y_start5 = [p8[1], k2p[1]]
        z_start5 = [p8[2], k2p[2]]
        x_start6 = [p8[0], k3p[0]]
        y_start6 = [p8[1], k3p[1]]
        z_start6 = [p8[2], k3p[2]]
        x_start7 = [p8[0], k4p[0]]
        y_start7 = [p8[1], k4p[1]]
        z_start7 = [p8[2], k4p[2]]
        x_start8 = [k1p[0], k2p[0], k3p[0], k4p[0], k1p[0]]
        y_start8 = [k1p[1], k2p[1], k3p[1], k4p[1], k1p[1]]
        z_start8 = [k1p[2], k2p[2], k3p[2], k4p[2], k1p[2]]

        ax1.plot3D(x_start , y_start, z_start, t, alpha=alpha)
        ax1.plot3D(x_start2 , y_start2, z_start2, 'r', alpha=alpha)
        ax1.plot3D(x_start3 , y_start3, z_start3, 'g', alpha=alpha)
        ax1.plot3D(x_start4 , y_start4, z_start4, 'k', alpha=alpha)
        ax1.plot3D(x_start5 , y_start5, z_start5, 'k', alpha=alpha)
        ax1.plot3D(x_start6 , y_start6, z_start6, 'k', alpha=alpha)
        ax1.plot3D(x_start7 , y_start7, z_start7, 'k', alpha=alpha)
        ax1.plot3D(x_start8 , y_start8, z_start8, 'k', alpha=alpha)

        ax2.plot(x_start, y_start, t, alpha=alpha)
        ax2.plot(x_start3, y_start3, 'g', alpha=alpha)
        ax2.plot(x_start8, y_start8, 'k', alpha=alpha)
        for p in self.table.actualPoints:
            if self.checkInFrame(p):
                x, y = self.point3DToCameraProjection(p)
                ax3.plot(x, y, 'g.', alpha=alpha)

