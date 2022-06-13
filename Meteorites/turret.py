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
from builtins import object
from copy import deepcopy

import numpy as np

from matrix import matrix

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = True
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

dT = 1

class Meteor(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.dx = 0
        self.dy = 0
        self.ddx = 0
        self.ddy = 0
        self.P = matrix([[.5, 0, 0, 0, 0, 0],
                        [0, .5, 0, 0, 0, 0],
                        [0, 0, 1000, 0, 0, 0],
                        [0, 0, 0, 1000, 0, 0],
                        [0, 0, 0, 0, 1000, 0],
                        [0, 0, 0, 0, 0, 1000]]) 

class Turret(object):
    """The laser used to defend against invading Meteorites."""

    def __init__(self, init_pos, arena_contains_fcn, max_angle_change,
                 initial_state):
        """Initialize the Turret."""
        self.x_pos = init_pos['x']
        self.y_pos = init_pos['y']
        self.arena_contains_fcn = arena_contains_fcn

        self.bounds_checker = arena_contains_fcn
        self.max_angle_change = max_angle_change
        self.meteors = dict()
        self.fire = 0

    def observe_and_estimate(self, meteorite_locations):
        """Observe the locations of the Meteorites.

        self is a reference to the current object, the Turret.
        meteorite_locations is a list of observations of meteorite locations.
        Each observation in meteorite_locations is a tuple (i, x, y), where i
        is the unique ID for an meteorite, and x, y are the x, y locations
        (with noise) of the current observation of that meteorite at this
        timestep. Only meteorites that are currently 'in-bounds' will appear in
        this list, so be sure to use the meteorite ID, and not the
        position/index within the list to identify specific meteorites. (The
        list may change in size as meteorites move in and out of bounds.)
        In this function, return the estimated meteorite locations as a tuple
        of (i, x, y) tuples, where i is a meteorite's ID, x is its
        x-coordinate, and y is its y-coordinate.
        """
        # TODO: Update the Turret's estimate of where the meteorites are
        # located at the current timestep and return the updated estimates
        F  =  matrix([[1., 0., 1., 0., 0.5, 0.],
                    [0., 1., 0., 1., 0., 0.5],
                    [0., 0., 1., 0., 1., 0.],
                    [0., 0., 0., 1., 0., 1.],
                    [0., 0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 0., 1.]]) 

        H =  matrix([[1., 0., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0., 0.]]) 

        R =  matrix([[.05, 0.],
                    [0., .05]]) 

        I = matrix([[1., 0., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0., 0.],
                    [0., 0., 1., 0., 0., 0.],
                    [0., 0., 0., 1., 0., 0.],
                    [0., 0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 0., 1.]])

        u = matrix([[0.], [0.], [0.], [0.], [0.], [0.]])
        res = []
        names = []
        delnames = []
        for each in meteorite_locations:
            name,x,y = each
            names.append(name)
            if not name in self.meteors:
                self.meteors[name] = Meteor(x,y)
            else:
                #Measurement Update
                met = self.meteors[name]
                curPos = matrix([[met.x],[met.y],[met.dx],[met.dy],[met.ddx],[met.ddy]])
                P = met.P
                #New Position
                Z = matrix([[x],[y]])
                Y = Z - (H * curPos)
                S = H * P * H.transpose() + R
                K = P * H.transpose() * S.inverse()
                curPos = curPos + (K * Y)
                P = (I - (K * H)) * P
                #Prediction
                predPos = F * curPos + u
                P = F*P*F.transpose()
                #Set in dictionary
                #print(predPos)
                self.meteors[name].x = predPos[0][0]
                self.meteors[name].y = predPos[1][0]
                self.meteors[name].dx = predPos[2][0]
                self.meteors[name].dy = predPos[3][0]
                self.meteors[name].ddx = predPos[4][0]
                self.meteors[name].ddy = predPos[5][0]
                self.meteors[name].P = P
            #print((name,self.meteors[name].x,self.meteors[name].y))
            res.append((name,self.meteors[name].x,self.meteors[name].y))
        #print(tuple(res))
        for metID in self.meteors:
            if not metID in names or metID == -1:
                delnames.append(metID)
        for iD in delnames:
            #print("deleted")
            del self.meteors[iD]
        return tuple(res)        

    def get_laser_action(self, current_aim_rad):
        """Return the laser's action; it can change its aim angle or fire.

        self is a reference to the current object, the Turret.
        current_aim_rad is the laser turret's current aim angle, in radians,
        provided by the simulation.

        The laser can aim in the range [0.0, pi].
        The maximum amount the laser's aim angle can change in a given timestep
        is self.max_angle_change radians. Larger change angles will be
        clamped to self.max_angle_change, but will keep the same sign as the
        returned desired angle change (e.g. an angle change of -3.0 rad would
        be clamped to -self.max_angle_change).
        If the laser is aimed at 0.0 rad, it will point horizontally to the
        right; if it is aimed at pi rad, it will point to the left.
        If the value returned from this function is the string 'fire' instead
        of a numerical angle change value, the laser will fire instead of
        moving.
        Returns: Float (desired change in laser aim angle, in radians), OR
        String 'fire' to fire the laser
        """
        # TODO: Update the change in the laser aim angle, in radians, based
        # on where the meteorites are currently, OR return 'fire' to fire the
        # laser at a meteorite
        if self.fire == 1:
            self.fire = 0
            return 'fire'
        else:
            closest = 20
            minx = 0
            miny = 0
            for each in self.meteors:
                met = self.meteors[each]
                y = met.y + 1
                x = met.x
                dist = x * x + y * y
                if y < closest:
                    closest = y
                    minx = x
                    miny = y
            try:
                angle = np.arctan(miny/minx)
            except:
                angle = current_aim_rad
            if angle < 0:
                angle = angle + np.pi
            #print(angle)
            if abs(angle - current_aim_rad) <= self.max_angle_change:
                #print('fiiire')
                self.fire = 1
        return angle - current_aim_rad  # or 'fire'


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith221).
    whoami = 'ezhang311'
    return whoami
