######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

#These import statements give you access to library functions which you may
# (or may not?) want to use.
from math import *
from glider import *
MINPARTICLES = 500
# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

#This is the function you will have to write for part A. 
#-The argument 'height' is a floating point number representing 
# the number of meters your glider is above the average surface based upon 
# atmospheric pressure. (You can think of this as height above 'sea level'
# except that Mars does not have seas.) Note that this sensor may be
# slightly noisy.
# This number will go down over time as your glider slowly descends.
#
#-The argument 'radar' is a floating point number representing the
# number of meters your glider is above the specific point directly below
# your glider based off of a downward facing radar distance sensor. Note that
# this sensor has random Gaussian noise which is different for each read.

#-The argument 'mapFunc' is a function that takes two parameters (x,y)
# and returns the elevation above "sea level" for that location on the map
# of the area your glider is flying above.  Note that although this function
# accepts floating point numbers, the resolution of your map is 1 meter, so
# that passing in integer locations is reasonable.
#
#
#-The argument OTHER is initially None, but if you return an OTHER from
# this function call, it will be passed back to you the next time it is
# called, so that you can use it to keep track of important information
# over time.
#
class particle:
   def __init__(self,x=0,y=0,heading = 0, sigma = pi/4):
      self.x = random.random() * 500 - 250 + x
      self.y = random.random() * 500 - 250 + y
      if not sigma == pi/4:
         self.orientation = random.random() *2*pi - pi
      else:
         self.orientation = random.gauss(heading,sigma)

      #print((self.x,self.y))

   def setXY(self, x, y, orient):
      self.x = float(x)
      self.y = float(y)
      self.orientation = float(orient)

   def move(self, turn, forward, turnsigma = pi/10, forwardsigma = 1.3):
      # turn, and add randomness to the turning command
      orientation = self.orientation + float(turn) + random.gauss(0,turnsigma)
      #orientation %= 2 * pi
      
      # move, and add randomness to the motion command
      dist = float(forward)
      x = self.x + (cos(orientation) * dist) + random.gauss(0,forwardsigma)
      y = self.y + (sin(orientation) * dist) + random.gauss(0,forwardsigma)
      """ x %= 5000    # cyclic truncate
      y %= 5000 """
      
      # set particle
      res = particle()
      res.setXY(x, y, orientation)
      #res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
      return res

   def Gaussian(self, mu, sigma, x):
      
      # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
      return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
   
   
   def measurement_prob(self, measurement, dist):
      
      # calculates how likely a distance should be
      # measurement is height of glider - radar. Dist is mapFunc(particle)
      
      prob = self.Gaussian(measurement, 3, dist)
      return prob

def estimate_next_pos(height, radar, mapFunc, OTHER=None):
   """Estimate the next (x,y) position of the glider."""

   #example of how to find the actual elevation of a point of ground from the map:
   actualElevation = mapFunc(5,4)

   # You must return a tuple of (x,y) estimate, and OTHER (even if it is NONE)
   # in this order for grading purposes.
   #
   xy_estimate = (0,0)  #Sample answer, (X,Y) as a tuple.

   #TODO - remove this canned answer which makes this template code
   #pass one test case once you start to write your solution.... 
   xy_estimate = (391.4400701739478, 1449.5287170970244) 
   
   if OTHER == None:
      OTHER = []
      N = 4000
      p = []
      for i in range(N):
         x = particle()
         p.append(x)
      OTHER.append(p)
      OTHER.append('')
      OTHER.append(())
      OTHER.append(False)
   else:
      N = len(OTHER[0])
      p = OTHER[0]
   
   # find weights
   w = []
   measurement = height - radar
   for i in range(N):
      dist = mapFunc(p[i].x,p[i].y)
      w.append(p[i].measurement_prob(measurement,dist))
   # resample
   p3 = []
   index = int(random.random() * N)
   beta = 0.0
   mw = max(w)
   #pos = w.index(mw)
   if N > MINPARTICLES:
      N = int(N * .9)
   for i in range(N):
      beta += random.random() * 2.0 * mw
      while beta > w[index]:
         beta -= w[index]
         index = (index + 1) % N
      p3.append(p[index])
   p = p3
   w = []
   measurement = height - radar
   for i in range(N):
      dist = mapFunc(p[i].x,p[i].y)
      w.append(p[i].measurement_prob(measurement,dist))
   #print(max(w))
   pos = w.index(max(w))
   res = p[pos]
   # move
   p2 = []
   if N < MINPARTICLES*2 and max(w) < .035:
      N = 4000
      for i in range(N):
         x = particle(p[pos].x,p[pos].y,OTHER[1],pi)
         p2.append(x)
   else:
      avgDist = []
      for i in range(N):
         if OTHER[1] == '':
            p2.append(p[i].move(0,5))
            distance = sqrt((p[i].x-p[pos].x)**2 + (p[i].y-p[pos].y)**2)
            #print(distance)
            avgDist.append(distance)
         else:
            ang = OTHER[1]-p[i].orientation
            p2.append(p[i].move(ang,5,pi/7,2.5))
      if len(avgDist) > 0 and sum(avgDist)/len(avgDist) < 20:
         #print(sum(avgDist)/len(avgDist))
         OTHER[3] = True
   p = p2
   # You may optionally also return a list of (x,y,h) points that you would like
   # the PLOT_PARTICLES=True visualizer to plot for visualization purposes.
   # If you include an optional third value, it will be plotted as the heading
   # of your particle.

   #optionalPointsToPlot = [ (1,1), (2,2), (3,3) ]  #Sample (x,y) to plot 
   #optionalPointsToPlot = [ (1,1,0.5),   (2,2, 1.8), (3,3, 3.2) ] #(x,y,heading)
   optionalPointsToPlot=[]
   """for i in range(N):
      plot = p[i]
      optionalPointsToPlot.append((plot.x,plot.y,plot.orientation)) """
   OTHER[0] = p
   #res = p[int(random.random() * N)]
   
   xy_estimate = (res.x,res.y)
   #print(xy_estimate)

   return xy_estimate, OTHER, optionalPointsToPlot


# This is the function you will have to write for part B. The goal in part B
# is to navigate your glider towards (0,0) on the map steering # the glider 
# using its rudder. Note that the Z height is unimportant.

#
# The input parameters are exactly the same as for part A.

def next_angle(height, radar, mapFunc, OTHER=None):
   
   xy,OTHER,optionalPointsToPlot = estimate_next_pos(height, radar, mapFunc, OTHER)
   #How far to turn this timestep, limited to +/-  pi/8, zero means no turn.
   steering_angle = 0.0
   if not OTHER == None:
      p = OTHER[0]
      #if len(p)<=MINPARTICLES:
      if OTHER[3] == True:
         w = []
         o = []
         measurement = height - radar
         for i in range(len(p)):
            dist = mapFunc(p[i].x,p[i].y)
            w.append(p[i].measurement_prob(measurement,dist))
            o.append(p[i].orientation)
         pos = w.index(max(w))
         x,y = xy
         targetAngle = atan2(y,x)
         if OTHER[1] == '':
            OTHER[1] = targetAngle
            #OTHER[1] = sum(o)/len(o)
         targetAngle += pi
         diff = abs(targetAngle - OTHER[1])
         if diff > pi:
            if targetAngle < 0:
               targetAngle += 2*pi
            else:
               targetAngle -= 2*pi
         steering_angle = diff % (pi/8.0)
         if targetAngle - OTHER[1] < 0:
            steering_angle = -steering_angle


         if OTHER[2]==():
            OTHER[1] = (steering_angle + OTHER[1])
         """ else:
            x0,y0 = OTHER[2]
            xdist = x-x0
            ydist = y-y0
            distMoved = sqrt((xdist)**2 + (ydist)**2)
            OTHER[1] = atan2(ydist,xdist) """
         """    if distMoved < 6:
               OTHER[1] = atan2(ydist,xdist)
            else:
               OTHER[1] = (steering_angle + OTHER[1]) """


         OTHER[1] = (steering_angle + OTHER[1])
         OTHER[2] = xy
         #print(OTHER[2])

   


      #curang = arctan


   # You may optionally also return a list of (x,y)  or (x,y,h) points that 
   # you would like the PLOT_PARTICLES=True visualizer to plot.
   #
   #optionalPointsToPlot = [ (1,1), (20,20), (150,150) ]  # Sample plot points 
   #return steering_angle, OTHER, optionalPointsToPlot

   return steering_angle, OTHER, optionalPointsToPlot

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith221).
    whoami = 'ezhang311'
    return whoami
