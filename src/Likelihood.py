from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import sys
import matplotlib.pyplot as plt
import copy
import sys
import os

# Get the root directory of the project
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Add the test/ directory explicitly
sys.path.append(os.path.join(project_root, "test"))

# Now, import the function from makeMeasurements.py
from makeMeasurements import *
from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot
from src.innerpoint import innerpoint
from src.EulerRotation import EulerRotation

from statsmodels.stats.outliers_influence import variance_inflation_factor  # check colinearity

"""
This Script includes a Likelihood function capable of finding the camera's parameters 
by taking pictures of the points and 

Author: Raul Penagos
Date: February 12th 2025
"""
class CustomStep:
    def __init__(self, step_sizes):
        self.step_sizes = np.array(step_sizes)  # Array con pasos diferentes por parámetro

    def __call__(self, x):
        perturbation = np.random.uniform(-self.step_sizes, self.step_sizes)
        return x + perturbation
    

class MyLikelihood(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      endog (array):  reference postions [X1, Y1, Z1, X2, Y2, Z2, ...] for each point
      exog (array): measurements [innerpoint, [X, Y]]   innerpoint(J1, J2, Z, Jz) 
      robot: robot cuyos parametros quiero optimizar
      **kwds: parametros iniciales (~ reales) a partir de los cuales minimizo
      """

      self.n = int(len(endog))
      self.endog = np.asarray(endog)
      J_points, camera_points = exog 
      exog = np.asarray([])

      print('N points: ', self.n)

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.rPoints = np.copy(self.endog)
      self.cPoints = np.copy(self.exog)

      print(self.rPoints.shape, self.cPoints.shape)

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihood, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0 = np.asarray([params[0], params[1], params[2]])
      # self.robot.camera.rotation0 = EulerRotation(params[0],params[1],params[2]) # pitch, roll y yaw
      # self.robot.camera.rotation0 = EulerRotation(params[3],0,0) # pitch, roll y yaw
      # self.robot.camera.rotation0 = EulerRotation(params[3], 0, 0) # pitch, roll y yaw

      self.robot.camera.rotation0 = EulerRotation(params[3], params[4], params[5]) # pitch, roll y yaw
      # self.robot.camera.sigmaCamera = params[6]
      # self.robot.camera.cx = params[7]
      # self.robot.camera.cy = params[8]

      # print('params', params)
      # print(self.xnames)

      chi2 = 0.0


      measurements2 = np.asarray([]) # I measure the J1 J2 X Y points 
      measurements3 = np.asarray([]) # I measure the xyz points 

      
      for point in self.rPoints:
         #  Loglike 1*************************************************
         # self.robot.cameraAim(point, 0)
         # print('Point:', point)
         # print('Camera Pointing: ', self.robot.cameraPointing())
         # j1, j2, z, jz = self.robot.currentPos.J1, self.robot.currentPos.J2, self.robot.currentPos.Z, self.robot.currentPos.Jz    
         # x, y = self.robot.point3DToCameraProjection(point)
         # print(x, y)
         # measurements = np.append(measurements, [j1, j2, z, jz, x, y])

         # Loglike2***************************************************
         self.robot.cameraAim(point, 0)
         # print(':)')
         j1, j2 = self.robot.currentPos.J1 , self.robot.currentPos.J2         
         x, y = self.robot.point3DToCameraProjection(point)
         measurements2 = np.append(measurements2, [j1, j2 , x, y])

         # Loglike2b***************************************************
         # self.robot.cameraAim(point, 0)
         # # print(':)')
         # j1, j2 = self.robot.currentPos.J1 , self.robot.currentPos.J2         
         # x, y = self.robot.point3DToCameraProjection(point)
         # measurements2 = np.append(measurements2, [j1, j2])

         # Loglike3*************************************************
         # self.robot.cameraAim(point, 0)
         # x, y, z = self.robot.cameraPointing()
         # measurements3 = np.append(measurements3, [x,y,z])
         

      # cameraProjectionToPoint3D  me da un punto x,y,z
      measurements2 = measurements2.reshape([int(len(measurements2)/4), 4])  
      measurements3 = measurements3.reshape([int(len(measurements3)/3), 3])  


      # Loglike2
      for i in range(0, self.n):
         new_measure = measurements2[i]
         cPoint_p = self.cPoints[i]
         cPoint = cPoint_p[0], cPoint_p[1], cPoint_p[4], cPoint_p[5]  # J1, J2, X, Y
         chi2 += np.linalg.norm(new_measure-cPoint)**2
      
      # Loglike2b
      # for i in range(0, self.n):
      #    new_measure = measurements2[i]
      #    cPoint_p = self.cPoints[i]
      #    cPoint = cPoint_p[0], cPoint_p[1]  # J1, J2
      #    chi2 += np.linalg.norm(new_measure-cPoint)**2
   
      #  Loglike3
      # for i in range(0, self.n):
      #    new_measure = measurements3[i]   # x y z medidas
      #    rPoint = self.rPoints[i]
         
      #    chi2 += np.linalg.norm(new_measure-rPoint)**2

      print(f'CHI2: {chi2}', end = "\r")

      return -chi2



   def fit(self, start_params=None, method='basinhopping', maxiter=10000, **kwargs):
      # methods = bfgs, lbfgs, nm, newton, powell, cg, ncg, basinhopping, minimize

      if start_params is None:
         # start_params =  [self.robot.camera.r0[0], self.robot.camera.r0[1], self.robot.camera.r0[2], 
         #                  self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi]
         # start_params =  [self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta,
         #                 self.robot.camera.rotation0.phi]
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1], self.robot.camera.r0[2], 
                         self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi]
        
      # Call the parent class's fit method

      if method == 'basinhopping':

         niter_success = 5
         step_sizes = [0.1, 0.1, 0.1, 0.002, 0.002, 0.002] 
         take_step = CustomStep(step_sizes)
         T = 0.8   # controla la probabilidad de aceptar soluciones peores para escapar de mínimos locales (0.1 - 1)

         bounds = [(-5, 5), (-5, 5), (-10, 10), (-0.35, 0.35), (-0.35, 0.35), (-0.35, 0.35)]
         minimizer_kwargs = {"method": "L-BFGS-B", "bounds": bounds}

         return super(MyLikelihood, self).fit(start_params=start_params, method=method, niter=maxiter, niter_success=niter_success,
                                             T=T,  stepsize=0.0, take_step=take_step, minimizer_kwargs = minimizer_kwargs, **kwargs)
      
      else:

         return super(MyLikelihood, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   

class Calibration():

   def __init__(self, real_robot, simul_robot):
      self.robot = real_robot 
      self.robot2 = simul_robot
      # Simulo las medidas reales, con robot1 real
      measurements, points, camera_measurements = make_measurements_camera_pointing(self.robot, False)      
      self.measurements = measurements
      self.camera_measurements = camera_measurements
      self.real_points = points
      n = len(self.real_points)

      print(camera_measurements)


   def calibrate(self):

      print('Calibrating...')

      # for i in range(n):
      #    print('a',self.real_points[i])
      #    print(self.robot.fromInnerToCartesian(self.measures[i][0], self.measures[i][1], 40))
      #    print(self.robot.fromInnerToCartesian(self.measures[i][0], self.measures[i][1], 0) - self.real_points[i])
      
      # No uso datos camara aun
      cal = MyLikelihood(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit()    

      # Mostrar resultados
      print("\n Full results summary:")
      print(results.summary(xname = ['x','y','z','psi', 'theta', 'phi']))


def main():
   
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
   # table.plotTable(ax1, ax2, 'g.')

   # Generate the camera  
   camera = Camera(x = 5.0, y = 0.0, z = 2.0, psi = np.pi/170, theta =  np.pi/160, phi = np.pi/180, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
   # psi = np.pi/170, theta =  np.pi/160, phi = np.pi/180

   # Generate the robot
   robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

   #  My guess for the actual robot
   robot2 = copy.deepcopy(robot)
   camera2 = Camera(8, 1, 2.5, 0.0, 0.0 , 0.0, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
   # camera2 = Camera(4.9, 0.57, 2.5, -0.06, -0.07 , 0.006, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)


   robot2.camera = camera2
   
   Cal_test = Calibration(robot, robot2)

   Cal_test.calibrate()

    






if __name__ == "__main__":
   main()

   
     