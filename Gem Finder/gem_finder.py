"""
 === Introduction ===

   The assignment is broken up into two parts.
   Part A:
        Create a SLAM implementation to process a series of landmark (gem) measurements and movement updates.
        The movements are defined for you so there are no decisions for you to make, you simply process the movements
        given to you.
        Hint: A planner with an unknown number of motions works well with an online version of SLAM.
    Part B:
        Here you will create the action planner for the robot.  The returned actions will be executed with the goal
        being to navigate to and extract a list of needed gems from the environment.  You will earn points by
        successfully extracting the list of gems from the environment. Extraction can only happen if within the
        minimum distance of 0.15.
        Example Actions (more explanation below):
            'move 3.14 1'
            'extract B 1.5 -0.2'
    Note: All of your estimates should be given relative to your robot's starting location.
    Details:
    - Start position
      - The robot will land at an unknown location on the map, however, you can represent this starting location
        as (0,0), so all future robot location estimates will be relative to this starting location.
    - Measurements
      - Measurements will come from gems located throughout the terrain.
        * The format is {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'D'}, ...}
      - Only gems that have not been collected and are within the horizon distance will return measurements.
    - Movements
      - Action: 'move 1.570963 1.0'
        * The robot will turn counterclockwise 90 degrees and then move 1.0
      - Movements are stochastic due to, well, it being a robot.
      - If max distance or steering is exceeded, the robot will not move.
    - Needed Gems
      - Provided as list of gem types: ['A', 'B', 'L', ...]
      - Although the gem names aren't real, as a convenience there are 26 total names, each represented by an
        upper case letter of the alphabet (ABC...).
      - Action: 'extract'
        * The robot will attempt to extract a specified gem from the current location..
      - When a gem is extracted from the terrain, it no longer exists in the terrain, and thus won't return a
        measurement.
      - The robot must be with 0.15 distance to successfully extract a gem.
      - There may be gems in the environment which are not required to be extracted.
    The robot will always execute a measurement first, followed by an action.
    The robot will have a time limit of 5 seconds to find and extract all of the needed gems.
"""

from typing import Dict, List
from matrix import *
from math import *

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')

class SLAM:
    """Create a basic SLAM module.
    """

    def __init__(self):
        """Initialize SLAM components here.
        """
        self.mu = matrix()
        self.Omega = matrix()

        self.Xi = matrix()
        self.locationList = []
        self.firstmeasurement = True
        self.bearing = 0

    # Provided Functions
    def get_coordinates_by_landmark_id(self, landmark_id: str):
        """
        Retrieves the x, y locations for a given landmark

        Args:
            landmark_id: The id for a processed landmark

        Returns:
            the coordinates relative to the robots frame with an initial position of 0.0
        """
        

        self.mu = self.Omega.inverse() * self.Xi
        i = self.locationList.index(landmark_id)
        m = 2 * (1 + i)

        return self.mu[m][0], self.mu[m+1][0]

    def process_measurements(self, measurements: Dict):
        """
        Process a new series of measurements.

        Args:
            measurements: Collection of measurements
                in the format {'landmark id':{'distance':0.0, 'bearing':0.0, 'type':'B'}, ...}

        Returns:
            x, y: current belief in location of the robot
        """
        if self.firstmeasurement:
            self.Omega.zero(2,2)
            self.Omega.value[0][0] = 1.0
            self.Omega.value[1][1] = 1.0

            self.Xi.zero(2, 1)
            self.firstmeasurement = False
        for landmarkID in measurements:
            if not landmarkID in self.locationList:
                self.locationList.append(landmarkID)
                length = self.Omega.dimx
                rowdim = list(range(length))
                self.Omega=self.Omega.expand(length + 2, length + 2, rowdim, rowdim)
                self.Xi=self.Xi.expand(length + 2, 1, rowdim, [0])
            #print("omega")
            #print(self.Omega)
            i = self.locationList.index(landmarkID)
            m = 2 * (1 + i)
            landmark = measurements[landmarkID]
            bearing = landmark['bearing'] + self.bearing
            xy = [landmark['distance'] * cos(bearing),landmark['distance'] * sin(bearing)]
            #measurement_noise = 0.1 * landmark['distance']
            #measurement_noise = abs(landmark['distance'] * 0.001)
            measurement_noise = 1
            for b in range(2):
                self.Omega.value[b][b] += 1.0/measurement_noise
                self.Omega.value[m+b][m+b] += 1.0/measurement_noise
                self.Omega.value[b][m+b] += -1.0/measurement_noise
                self.Omega.value[m+b][b] += -1.0/measurement_noise

                self.Xi.value[b][0] += -xy[b] / measurement_noise
                self.Xi.value[m+b][0] += xy[b] / measurement_noise
        #print("omega")
        #print(self.Omega)
        self.mu = self.Omega.inverse() * self.Xi

        return self.mu[0][0], self.mu[1][0]

    def process_movement(self, steering: float, distance: float):
        """
        Process a new movement.

        Args:
            steering: amount to turn
            distance: distance to move

        Returns:
            x, y: current belief in location of the robot
        """
        #for movement??
        dim = len(self.locationList)
        dim = 2* (1+dim)
        list1 = [0,1] + list(range(4,dim+2))
        self.Omega = self.Omega.expand(dim+2, dim+2, list1, list1)
        self.Xi = self.Xi.expand(dim+2,1,list1,[0])
        #motion_noise = abs(0.001 * distance)
        motion_noise = 0.1
        steering += self.bearing
        xy = [distance * cos(steering), distance * sin(steering)]


        for b in range(4):
            self.Omega.value[b][b] += 1.0/motion_noise
        for b in range(2):
            self.Omega.value[b][b+2] += -1.0 / motion_noise
            self.Omega.value[b+2][b] += -1.0 / motion_noise
            self.Xi.value[b][0] += -xy[b] / motion_noise
            self.Xi.value[b+2][0] += xy[b] / motion_noise
        
        newlist = range(2, len(self.Omega.value))
        a = self.Omega.take([0,1], newlist)
        b = self.Omega.take([0,1])
        c = self.Xi.take([0,1], [0])
        self.Omega = self.Omega.take(newlist) - a.transpose() * b.inverse() * a 
        self.Xi = self.Xi.take(newlist, [0]) - a.transpose() * b.inverse() * c 
        self.mu = self.Omega.inverse() * self.Xi
        self.bearing = steering
        return self.mu[0][0], self.mu[1][0]


class GemExtractionPlanner:
    """
    Create a planner to navigate the robot to reach and extract all the needed gems from an unknown start position.
    """

    def __init__(self, max_distance: float, max_steering: float):
        """
        Initialize your planner here.

        Args:
            max_distance: the max distance the robot can travel in a single move.
            max_steering: the max steering angle the robot can turn in a single move.
        """
        self.SLAM = SLAM()
        self.max_distance = max_distance
        self.max_steering = max_steering
        self.x = 0
        self.y = 0
        self.extract = False
        self.spiral = 0

    def next_move(self, needed_gems: List[str], measurements: Dict):
        """Next move based on the current set of measurements.
        Args:
            needed_gems: List of gems remaining which still need to be found and extracted.
            measurements: Collection of measurements from gems in the area.
                                {'landmark id': {
                                                    'distance': 0.0,
                                                    'bearing' : 0.0,
                                                    'type'    :'B'
                                                },
                                ...}
        Return: action: str, points_to_plot: dict [optional]
            action (str): next command to execute on the robot.
                allowed:
                    'move 1.570963 1.0'  - Turn left 90 degrees and move 1.0 distance.
                    'extract B 1.5 -0.2' - [Part B] Attempt to extract a gem of type B from your current location.
                                           This will succeed if the specified gem is within the minimum sample distance.
            points_to_plot (dict): point estimates (x,y) to visualize if using the visualization tool [optional]
                            'self' represents the robot estimated position
                            <landmark_id> represents the estimated position for a certain landmark
                format:
                    {
                        'self': (x, y),
                        '<landmark_id_1>': (x1, y1),
                        '<landmark_id_2>': (x2, y2),
                        ....
                    }
        """
        self.x,self.y = self.SLAM.process_measurements(measurements)
        mindist = 999999999999999
        closest = ''
        closesttype  = ''
        bearing = -pi/2
        distance = self.max_distance
        points_to_plot = {}
        for landmarkID in measurements:
            landmark = measurements[landmarkID]
            points_to_plot[landmarkID] = self.SLAM.get_coordinates_by_landmark_id(landmarkID)
            if landmark['distance'] < mindist and landmark['type'] in needed_gems:
                mindist = landmark['distance']
                closest = landmarkID
                closesttype = landmark['type']
        if closest == '':
            bearing = min(bearing + self.spiral * 0.02, -0.3927)
            self.x,self.y = self.SLAM.process_movement(bearing,self.max_distance)
            points_to_plot['self'] = (self.x,self.y)
            self.spiral +=1
            #print(self.spiral)
            #print(bearing)
            movestr = 'move ' + str(bearing) + ' ' + str(distance)
            return movestr, points_to_plot
        else:
            self.spiral = 0
            lx,ly = self.SLAM.get_coordinates_by_landmark_id(closest)
            if self.extract:
                self.extract = False
                #print("extracted")
                return 'extract ' + closesttype + ' ' + str(lx) + ' ' + str(ly), points_to_plot
            else:
                #bearing = atan2(ly - self.y, lx - self.x)
                bearing = float(measurements[closest]['bearing'])
                #print(measurements[closest]['type'])
                if bearing > pi/2:
                    bearing = pi/2
                    distance = 0.5
                elif bearing < -pi/2:
                    bearing = -pi/2
                    distance = 0.5
                if sqrt((lx-self.x)*(lx-self.x) + (ly-self.y)*(ly-self.y)) < self.max_distance:
                    self.extract = True
                    distance = sqrt((lx-self.x)*(lx-self.x) + (ly-self.y)*(ly-self.y))
        self.x,self.y = self.SLAM.process_movement(bearing,distance)
        points_to_plot['self'] = (self.x,self.y)
        movestr = 'move ' + str(bearing) + ' ' + str(distance)
        #print(movestr)
        return movestr, points_to_plot

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith221).
    whoami = 'ezhang311'
    return whoami
