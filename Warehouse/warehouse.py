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


import math

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class DeliveryPlanner_PartA:
    """
    Required methods in this class are:
    
      plan_delivery(self, debug = False) which is stubbed out below.  
        You may not change the method signature as it will be called directly 
        by the autograder but you may modify the internals as needed.
    
      __init__: which is required to initialize the class.  Starter code is 
        provided that initializes class variables based on the definitions in
        testing_suite_partA.py.  You may choose to use this starter code
        or modify and replace it based on your own solution
    
    The following methods are starter code you may use for part A.  
    However, they are not required and can be replaced with your
    own methods.
    
      _set_initial_state_from(self, warehouse): creates structures based on
          the warehouse and todo definitions and initializes the robot
          location in the warehouse
    
      _search(self, debug=False): Where the bulk of the A* search algorithm
          could reside.  It should find an optimal path from the robot
          location to a goal.  Hint:  you may want to structure this based
          on whether looking for a box or delivering a box.
  
    """

    ## Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, todo):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self._set_initial_state_from(warehouse)

        self.delta = [[-1, 0],  # north
                      [0, -1],  # west
                      [1, 0],  # south
                      [0, 1],  # east
                      [-1, -1],  # northwest (diag)
                      [-1, 1],  # northeast (diag)
                      [1, 1],  # southeast (diag)
                      [1, -1]]  # southwest (diag)

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        # Can use this for a visual debug
        self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        # self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]

    ## state parsing and initialization function from testing_suite_partA.py
    def _set_initial_state_from(self, warehouse):
        """Set initial state.
        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '*'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

        self.robot_position = self.dropzone
        self.box_held = None

    def _search(self, end, debug=False):
        """
        This method should be based on lesson modules for A*, see Search, Section 12-14.
        The bulk of the search logic should reside here, should you choose to use this starter code.
        Please condition any printout on the debug flag provided in the argument.  
        You may change this function signature (i.e. add arguments) as 
        necessary, except for the debug argument which must remain with a default of False
        """

        # get a shortcut variable for the warehouse (note this is just a view no copying)
        grid = self.warehouse_state

        # Find and fill in the required moves per the instructions - example moves for test case 1
        """ moves = ['move w',
                 'move nw',
                 'lift 1',
                 'move se',
                 'down e',
                 'move ne',
                 'lift 2',
                 'down s'] """
        # The following code is from: https://gatech.instructure.com/courses/192324/pages/13-implement-a-star-answer?module_item_id=1634164

        x,y = self.robot_position
        sx,sy = self.robot_position
        print("robotposition")
        print(self.robot_position)
        print("dropzone")
        print(self.dropzone)
        ex,ey = end
        print("end")
        print(end)
        h = abs(x-ex) + abs(y-ey)
        g = 0
        f = g+h

        closed_list = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
        closed_list[sx][sy] = 1

        expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
        action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

        open_list = [[f, g, h, x, y]]

        found = False
        resign = False
        count = 0

        while not found and not resign:
            print(open_list)
            if len(open_list) == 0:
                resign = True
                print("RESIGNED")
                for i in range(len(grid)):
                    print(grid[i])
                return []
            else:
                open_list.sort()
                open_list.reverse()
                next_node = open_list.pop()
                x = next_node[3]
                y = next_node[4]
                g = next_node[1]
                expand[x][y] = count
                count += 1
                if x == ex and y == ey:
                    found = True
                    print("FOUNDDDD")
                    for i in range(len(grid)):
                        print(grid[i])
                else:
                    for i in range(len(self.delta)):
                        x2 = x + self.delta[i][0]
                        y2 = y + self.delta[i][1]
                        cost = self.delta_cost[i]
                        if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                            if closed_list[x2][y2] == 0 and (grid[x2][y2] == '.' or (x2,y2) == end or grid[x2][y2] == '@' or grid[x2][y2] == '*'):
                                g2 = g + cost
                                h2 = abs(x2-ex) + abs(y2-ey)
                                f2 = g2 + h2
                                open_list.append([f2, g2, h2, x2, y2])
                                closed_list[x2][y2] = 1
                                action[x2][y2] = i
                                if (x2,y2) == end:
                                    self.robot_position = (x,y)
                                """ for i in range(len(action)):
                                    print(action[i]) """
        #end copied code
        x = ex
        y = ey
        moves = []
        while x != sx or y != sy:
            x2 = x - self.delta[action[x][y]][0]
            y2 = y - self.delta[action[x][y]][1]
            moves.append(self.delta_directions[action[x][y]])
            x = x2
            y = y2
        moves.reverse()
        grid[ex][ey] = '.'
        return moves

    def plan_delivery(self, debug=True):
        """
        plan_delivery() is required and will be called by the autograder directly.  
        You may not change the function signature for it.
        Add logic here to find the moves.  You may use the starter code provided above
        in any way you choose, but please condition any printouts on the debug flag
        """

        # Find the moves - you may add arguments and change this logic but please leave
        # the debug flag in place and condition all printouts on it.

        # You may wish to break the task into one-way paths, like this:
        #
        #    moves_to_1   = self._search( ..., debug=debug )
        #    moves_from_1 = self._search( ..., debug=debug )
        #    moves_to_2   = self._search( ..., debug=debug )
        #    moves_from_2 = self._search( ..., debug=debug )
        #    moves        = moves_to_1 + moves_from_1 + moves_to_2 + moves_from_2
        #
        # If you use _search(), you may need to modify it to take some
        # additional arguments for starting location, goal location, and
        # whether to pick up or deliver a box.
        moves = []
        print(self.todo)
        for i in range(len(self.todo)):
            movesTo = self._search(self.boxes[self.todo[i]])
            for x in range(0,len(movesTo)-1):
                movesTo[x]  = "move " + movesTo[x]
            print("movesTo")
            last = movesTo[-1]
            movesTo[-1] = "lift " + self.todo[i]
            if self.robot_position == self.dropzone:
                movesTo.append("move " + last)
                self.robot_position = self.boxes[self.todo[i]]
            print(movesTo)
            movesFrom = self._search(self.dropzone)
            print("xxxxxx")
            for y in range(0,len(movesFrom)-1):
                movesFrom[y]  = "move " + movesFrom[y]
            movesFrom[-1] = "down " + movesFrom[-1]
            print(movesFrom)
            moves += movesTo + movesFrom

        if debug:
            for i in range(len(moves)):
                print(moves[i])

        return moves


class DeliveryPlanner_PartB:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partB.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part B.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, debug=False): Where the bulk of the dynamic
            programming (DP) search algorithm could reside.  It should find
            an optimal path from the robot location to a goal.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost

        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # up left (diag)
                      [-1, 1],  # up right (diag)
                      [1, 1],  # dn right (diag)
                      [1, -1]]  # dn left (diag)

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        # Use this for a visual debug
        self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        # self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]

    # state parsing and initialization function from testing_suite_partA.py
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '*'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def _find_policy(self, goal, pickup_box=True, debug=False):
        """
        This method should be based on lesson modules for Dynamic Programming,
        see Search, Section 15-19 and Problem Set 4, Question 5.  The bulk of
        the logic for finding the policy should reside here should you choose to
        use this starter code.  Please condition any printout on the debug flag
        provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################

        # get a shortcut variable for the warehouse (note this is just a view it does not make a copy)
        grid = self.warehouse_state
        grid_costs = self.warehouse_cost

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1

        # The following code is from: https://gatech.instructure.com/courses/192324/pages/19-optimum-policy-answer?module_item_id=1634188
        value = [[10000 for row in range(len(grid[0]))] for col in range(len(grid))]
        policy = [['-1' for row in range(len(grid[0]))] for col in range(len(grid))]
        change = True
        gx,gy = goal

        while change:
            change = False

            for x in range(len(grid)):
                for y in range(len(grid[0])):
                    """ if gx == x and gy == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            #what to write for policy??
                            change = True

                    elif grid[x][y] == 0:
                        for a in range(len(self.delta)):
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]
                            cost = self.delta_cost[a]
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and (grid[x2][y2] == '.' or (x2,y2) == (gx,gy) or grid[x2][y2] == '@' or grid[x2][y2] == '*'):
                                v2 = value[x2][y2] + cost

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    if x2 == gx and y2 == gy:
                                        if pickup_box:
                                            policy[x][y] = 'lift 1'
                                        else:
                                            policy[x][y] = 'down ' + self.delta_name[a]
                                    else:
                                        policy[x][y] = 'move ' + self.delta_name[a] """
                    if gx == x and gy == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            if pickup_box:
                                policy[x][y] = 'B'
                            else:
                                valid = False
                                mincost = 100000
                                minD = 0
                                for d in range(len(self.delta)):
                                    x3 = x + self.delta[d][0]
                                    y3 = y + self.delta[d][1]
                                    if x3 >= 0 and x3 < len(grid) and y3 >= 0 and y3 < len(grid[0]) and (grid[x3][y3] == '.' or grid[x3][y3] == '1'):
                                        cost = self.delta_cost[d] + grid_costs[x3][y3]
                                        if cost < mincost:
                                            mincost = cost
                                            minD = d
                                policy[x][y] = "move " + self.delta_directions[minD]
                            #what to write for policy??
                            change = True
                    elif grid[x][y] == '.' or grid[x][y] == '1' or grid[x][y] == '*':
                        for a in range(len(self.delta)):
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]
                            cost = self.delta_cost[a]
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and (grid[x2][y2] == '.' or  (gx,gy) == (x2,y2) or grid[x2][y2] == '1' or grid[x2][y2] == '*'):
                                v2 = value[x2][y2] + cost + grid_costs[x2][y2]

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                # end copied code
                                    if x2 == gx and y2 == gy:
                                        if pickup_box:
                                            policy[x][y] = 'lift 1'
                                        else:
                                            policy[x][y] = 'down ' + self.delta_directions[a]
                                    else:
                                        policy[x][y] = 'move ' + self.delta_directions[a]
        print("\nGRID:")
        for i in range(len(grid)):
            print(grid[i])
        return policy

    def plan_delivery(self, debug=True):
        """
        plan_delivery() is required and will be called by the autograder directly.  
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################

        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        goal = self.boxes['1']
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
        goal = self.dropzone
        deliver_policy = self._find_policy(goal, pickup_box=False, debug=debug)

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(deliver_policy)):
                print(deliver_policy[i])

        return (to_box_policy, deliver_policy)


class DeliveryPlanner_PartC:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partC.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part C.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, debug=False): Where the bulk of your algorithm
            could reside.  It should find an optimal policy to a goal.
            Remember that actions are stochastic rather than deterministic.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo, p_outcomes):

        self.todo = todo
        self.boxes_delivered = []
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.p_outcomes = p_outcomes

        self.delta = [
            [-1, 0],  # go up
            [-1, -1],  # up left (diag)
            [0, -1],  # go left
            [1, -1],  # dn left (diag)
            [1, 0],  # go down
            [1, 1],  # dn right (diag)
            [0, 1],  # go right
            [-1, 1],  # up right (diag)]
        ]

        self.delta_directions = ["n", "nw", "w", "sw", "s", "se", "e", "ne"]

        # Use this for a visual debug
        self.delta_name = ['🡑', '🡔', '🡐', '🡗', '🡓', '🡖', '🡒', '🡕']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST, ]

    # state parsing and initialization function from testing_suite_partA.py
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '*'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def _find_policy(self, goal, pickup_box=True, debug=False):
        """
        You are free to use any algorithm necessary to complete this task.
        Some algorithms may be more well suited than others, but deciding on the
        algorithm will allow you to think about the problem and understand what
        tools are (in)adequate to solve it. Please condition any printout on the
        debug flag provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################

        # get a shortcut variable for the warehouse (note this is just a view it does not make a copy)
        grid = self.warehouse_state
        grid_costs = self.warehouse_cost

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1

        # The following code is from: https://gatech.instructure.com/courses/192324/pages/19-optimum-policy-answer?module_item_id=1634188
        value = [[1000 for row in range(len(grid[0]))] for col in range(len(grid))]
        policy = [['-1' for row in range(len(grid[0]))] for col in range(len(grid))]
        change = True
        gx,gy = goal

        while change:
            change = False

            for x in range(len(grid)):
                for y in range(len(grid[0])):
                    """if gx == x and gy == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            #what to write for policy??
                            change = True

                    elif grid[x][y] == 0:
                        for a in range(len(self.delta)):
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]
                            cost = self.delta_cost[a]
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and (grid[x2][y2] == '.' or (x2,y2) == (gx,gy) or grid[x2][y2] == '@' or grid[x2][y2] == '*'):
                                v2 = value[x2][y2] + cost

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    if x2 == gx and y2 == gy:
                                        if pickup_box:
                                            policy[x][y] = 'lift 1'
                                        else:
                                            policy[x][y] = 'down ' + self.delta_name[a]
                                    else:
                                        policy[x][y] = 'move ' + self.delta_name[a] """
                    if gx == x and gy == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            if pickup_box:
                                policy[x][y] = 'B'
                            else:
                                valid = False
                                mincost = 100000
                                minD = 0
                                for d in range(len(self.delta)):
                                    x3 = x + self.delta[d][0]
                                    y3 = y + self.delta[d][1]
                                    if x3 >= 0 and x3 < len(grid) and y3 >= 0 and y3 < len(grid[0]) and (grid[x3][y3] == '.' or grid[x3][y3] == '1'):
                                        cost = self.delta_cost[d] + grid_costs[x3][y3]
                                        if cost < mincost:
                                            mincost = cost
                                            minD = d
                                policy[x][y] = "move " + self.delta_directions[minD]
                            #what to write for policy??
                            change = True
                    elif grid[x][y] == '.' or grid[x][y] == '1' or grid[x][y] == '*':
                        for a in range(len(self.delta)):
                            v2 = 0
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and (grid[x2][y2] == '.' or  (gx,gy) == (x2,y2) or grid[x2][y2] == '1' or grid[x2][y2] == '*'):
                                for i in range(-2,3):
                                    a2 = (a + i) % len(self.delta)
                                    x3 = x + self.delta[a2][0]
                                    y3 = y + self.delta[a2][1]
                                    if i == 0:
                                        p2 = self.p_outcomes['success']
                                    elif i == -1 or i == 1:
                                        p2 = self.p_outcomes['fail_diagonal']
                                    else:
                                        p2 = self.p_outcomes['fail_orthogonal']
                                    cost = self.delta_cost[a2]
                                    if x3 >= 0 and x3 < len(grid) and y3 >= 0 and y3 < len(grid[0]) and (grid[x3][y3] == '.' or  (gx,gy) == (x3,y3) or grid[x3][y3] == '1' or grid[x3][y3] == '*'):
                                        v2 += p2*(value[x3][y3] + grid_costs[x3][y3] + cost)
                                    else:
                                        v2 += p2 * (400+ 100 + cost)

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                # end copied code
                                    if x2 == gx and y2 == gy:
                                        if pickup_box:
                                            policy[x][y] = 'lift 1'
                                        else:
                                            policy[x][y] = 'down ' + self.delta_directions[a]
                                    else:
                                            policy[x][y] = 'move ' + self.delta_directions[a]
        print("\nGRID:")
        for i in range(len(grid_costs)):
            print(grid_costs[i])
        """ grid = self.warehouse_state
        grid_costs = self.warehouse_cost

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1

        # The following code is from: https://gatech.instructure.com/courses/192324/pages/19-optimum-policy-answer?module_item_id=1634188
        value = [[10000 for row in range(len(grid[0]))] for col in range(len(grid))]
        policy = [['-1' for row in range(len(grid[0]))] for col in range(len(grid))]
        change = True
        gx,gy = goal

        while change:
            change = False

            for x in range(len(grid)):
                for y in range(len(grid[0])):
                    """ """if gx == x and gy == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            #what to write for policy??
                            change = True

                    elif grid[x][y] == 0:
                        for a in range(len(self.delta)):
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]
                            cost = self.delta_cost[a]
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and (grid[x2][y2] == '.' or (x2,y2) == (gx,gy) or grid[x2][y2] == '@' or grid[x2][y2] == '*'):
                                v2 = value[x2][y2] + cost

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    if x2 == gx and y2 == gy:
                                        if pickup_box:
                                            policy[x][y] = 'lift 1'
                                        else:
                                            policy[x][y] = 'down ' + self.delta_name[a]
                                    else:
                                        policy[x][y] = 'move ' + self.delta_name[a] """"""
                    if gx == x and gy == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            if pickup_box:
                                policy[x][y] = 'B'
                            else:
                                valid = False
                                mincost = 100000
                                minD = 0
                                for d in range(len(self.delta)):
                                    x3 = x + self.delta[d][0]
                                    y3 = y + self.delta[d][1]
                                    if x3 >= 0 and x3 < len(grid) and y3 >= 0 and y3 < len(grid[0]) and (grid[x3][y3] == '.' or grid[x3][y3] == '1'):
                                        cost = self.delta_cost[d] + grid_costs[x3][y3]
                                        if cost < mincost:
                                            mincost = cost
                                            minD = d
                                policy[x][y] = "move " + self.delta_directions[minD]
                            #what to write for policy??
                            change = True
                    elif grid[x][y] == '.' or grid[x][y] == '1' or grid[x][y] == '*':
                        for a in range(len(self.delta)):
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]
                            cost = self.delta_cost[a]
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and (grid[x2][y2] == '.' or  (gx,gy) == (x2,y2) or grid[x2][y2] == '1' or grid[x2][y2] == '*'):
                                v2 = value[x2][y2] + cost + grid_costs[x2][y2]

                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                # end copied code
                                    if x2 == gx and y2 == gy:
                                        if pickup_box:
                                            policy[x][y] = 'lift 1'
                                        else:
                                            policy[x][y] = 'down ' + self.delta_directions[a]
                                    else:
                                        policy[x][y] = 'move ' + self.delta_directions[a]
        print("\nGRID:")
        for i in range(len(grid)):
            print(grid[i]) """
        return policy

    def plan_delivery(self, debug=True):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################

        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        goal = self.boxes['1']
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
        goal = self.dropzone
        to_zone_policy = self._find_policy(goal, pickup_box=False, debug=debug)

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        to_box_values = None
        to_zone_values = None
        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith221).
    whoami = 'ezhang311'
    return whoami


if __name__ == "__main__":
    """ 
    You may execute this file to develop and test the search algorithm prior to running 
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will not be called by the autograder

    # Testing for Part A
    # testcase 1
    print('\nTesting for part A:')
    warehouse = ['1#2',
                 '.#.',
                 '..@']

    todo = ['1', '2']

    partA = DeliveryPlanner_PartA(warehouse, todo)
    partA.plan_delivery(debug=True)

    # Testing for Part B
    # testcase 1
    print('\nTesting for part B:')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[0, 5, 2],
                      [10, math.inf, 2],
                      [2, 10, 2]]

    todo = ['1']

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    partB.plan_delivery(debug=True)

    # Testing for Part C
    # testcase 1
    print('\nTesting for part C:')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[13, 5, 6],
                      [10, math.inf, 2],
                      [2, 11, 2]]

    todo = ['1']

    p_outcomes = {'success': .70,
                  'fail_diagonal': .1,
                  'fail_orthogonal': .05, }

    partC = DeliveryPlanner_PartC(warehouse, warehouse_cost, todo, p_outcomes)
    partC.plan_delivery(debug=True)
