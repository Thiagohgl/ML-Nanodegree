import numpy as np
import numpy.matlib
class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''


        self.heading = 'up'
        self.maze_dim = maze_dim
        self.location = [0, 0]
        self.goal = [self.maze_dim/2 - 1, self.maze_dim/2]#[0,maze_dim-1]#[maze_dim/2 - 1, maze_dim/2]
        self.is_exploring = True

        # Barriers to map the obstacles
        self.Barries_up = np.zeros((maze_dim,maze_dim)) # Ones are barriers
        self.Barries_left = np.zeros((maze_dim,maze_dim))
        self.Barries_down = np.zeros((maze_dim,maze_dim))
        self.Barries_right = np.zeros((maze_dim,maze_dim))

        # Put all barriers in a list
        self.barriers = []
        self.barriers.append(self.Barries_up)
        self.barriers.append(self.Barries_left)
        self.barriers.append(self.Barries_down)
        self.barriers.append(self.Barries_right)

        # DO A MAPPING TO THE MOVEMENT POSITION TO ACTUAL GRID POSITION
        Dimension_vector_mappingX = np.arange(0, maze_dim, 1)
        self.Dimension_vector_mappingY = np.transpose(Dimension_vector_mappingX)[::-1]

        self.movements = [[0,1] # Up
                         ,[-1,0] # Left
                         ,[0,-1] # Down
                         ,[1,0]] # Right

        self.Representation = ['D','R','U','L'] # To plot in the path
        self.PathMap = np.chararray((maze_dim, maze_dim)) # Mattrix where the path will be saved
        self.PathMap[:] = 'o'

        # Make Heuristic to A*
        x = self.goal[0]
        y = self.goal[1]
        self.heuristic =  np.zeros((maze_dim,maze_dim))
        self.heuristic[0,0] = maze_dim - 1

        for idxX in range(1,maze_dim):
            self.heuristic[idxX, 0] = np.absolute(self.goal[0] - idxX) #+ maze_dim/2
        for idxY in range(1,maze_dim):
            self.heuristic[:, idxY] = self.heuristic[:, 0] + np.absolute((self.goal[1]  - idxY))

        # If no heuristic, set all values to 0
        #self.heuristic = np.zeros((maze_dim, maze_dim))

        # Variable to specify the exploration run
        self.run = 0

        # Counter of steps
        self.steps_train = 0
        self.steps_real = 0

        # Function to calculate the new heuristic depending of the goal
    def Get_heuristic(self):
        for idxX in range(1,self.maze_dim):
            self.heuristic[idxX, 0] = np.absolute(self.goal[0] - idxX) #+ maze_dim/2
        for idxY in range(1,self.maze_dim):
            self.heuristic[:, idxY] = self.heuristic[:, 0] + np.absolute((self.goal[1]  - idxY))

        # If no heuristic
        #self.heuristic = np.zeros((self.maze_dim, self.maze_dim))

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        #sensors_np = np.array(sensors)
        if True:#self.is_exploring:

            if self.location == self.goal and self.goal != (0,0):
                # Re start the goal to go back with another way
                self.run += 1
                if self.run == 6:
                    self.goal = [0,0]
                if  self.run == 3 or self.run == 5 or self.run == 7: #self.run == 1
                    self.goal = [self.maze_dim/2 - 1, self.maze_dim/2]#[self.maze_dim-1,self.maze_dim-1]
                if self.run == 1:
                    self.goal = [0,self.maze_dim-1]
                if self.run == 2:
                    self.goal = [self.maze_dim-1,0]
                if self.run == 4:
                    self.goal = [self.maze_dim-1,self.maze_dim-1]


                # Get actual heuristic
                self.Get_heuristic()

            # If back to start, begin the second run and reset everything
            if self.location == self.goal:

                # Re start parameters
                self.location = [0,0]
                self.goal = [self.maze_dim/2 - 1, self.maze_dim/2]
                self.heading = 'up'
                self.is_exploring = False
                output = ('Reset', 'Reset')
                self.Get_heuristic()
                self.PathMap = np.chararray((self.maze_dim, self.maze_dim))
                self.PathMap[:] = '-'
                self.PathMap[self.Dimension_vector_mappingY[self.maze_dim/2] , self.maze_dim/2-1] = 'X'
                return output

            #print 'Sensor' + str(sensors)
            #print 'self.heading' + str(self.heading)
            ####### Use sensor and position data to fill the maze barriers#######################

            # Use the sensor data to map the walls of the maze
            # X coordinate is Y barrier place and Y coordinate is X barrier place with mapping
            if self.heading == 'up':

                self.barriers[0][self.Dimension_vector_mappingY[self.location[1]]-(sensors[1]),self.location[0]] = 1
                self.barriers[1][self.Dimension_vector_mappingY[self.location[1]],self.location[0]-( sensors[0])] = 1
                self.barriers[3][ self.Dimension_vector_mappingY[self.location[1]] ,self.location[0]+ sensors[2]] = 1

            if self.heading == 'left':

                self.barriers[0][self.Dimension_vector_mappingY[self.location[1]]- sensors[2],self.location[0] ] = 1
                self.barriers[1][self.Dimension_vector_mappingY[self.location[1]],self.location[0]-(sensors[1])] = 1
                self.barriers[2][self.Dimension_vector_mappingY[self.location[1]-( sensors[0])],self.location[0]] = 1
            if self.heading == 'down':

                self.barriers[3][self.Dimension_vector_mappingY[self.location[1]],self.location[0]+( sensors[0])] = 1
                self.barriers[1][self.Dimension_vector_mappingY[self.location[1]],self.location[0]- sensors[2] ] = 1
                self.barriers[2][self.Dimension_vector_mappingY[self.location[1]-(sensors[1])],self.location[0]] = 1

            if self.heading == 'right':

                self.barriers[0][self.Dimension_vector_mappingY[self.location[1]]-( sensors[0]),self.location[0]] = 1
                self.barriers[3][self.Dimension_vector_mappingY[self.location[1]],self.location[0]+(sensors[1])] = 1
                self.barriers[2][self.Dimension_vector_mappingY[self.location[1] - sensors[2]],self.location[0]] = 1


            #for idx in self.Barries_left:
                #print idx

            ################################# DECIDE WHICH MOVEMENT TO TAKE ########################################

            #Random movement
            #Direction, Intensity =  self.getNextDirectionRandom()

            # A*
            Direction,Intensity = self.getNextDirectionAstar(self.goal)

            #print 'Direction: ' + str(Direction) + '. Orientation: ' + str(self.heading)

            movement,rotation = self.Convert_stage(Direction,Intensity)


            print 'Position: ' + str(self.location) + '. Orientation: ' + str(self.heading) + '. Goal: '+str(self.goal)
            #print 'Direction: ' + str(Direction) + '. Rotation: ' + str(rotation) + '. Movement ' + str(movement)

            ########################### HEADING AND LOCATION AFTER MOVEMENT####################
            # handle the heading
            if self.heading == 'up' and rotation == 90:
                self.heading = 'right'

            elif self.heading == 'up' and rotation == -90:
                self.heading = 'left'

            elif self.heading == 'right' and rotation == 90:
                self.heading = 'down'
            elif self.heading == 'right' and rotation == -90:
                self.heading = 'up'

            elif self.heading == 'down' and rotation == 90:
                self.heading = 'left'
            elif self.heading == 'down' and rotation == -90:
                self.heading = 'right'

            elif self.heading == 'left' and rotation == 90:
                self.heading = 'up'
            elif self.heading == 'left' and rotation == -90:
                self.heading = 'down'
            # Handle the location
            print 'New Heading:' + str(self.heading)
            # Up
            if self.heading == 'up':
                self.location[0] += self.movements[0][0]*movement
                self.location[1] += self.movements[0][1]*movement

            if self.heading == 'down':
                self.location[0] += self.movements[2][0]*movement
                self.location[1] += self.movements[2][1]*movement
            #Right
            if self.heading == 'right':

                self.location[0] += self.movements[3][0]*movement
                self.location[1] += self.movements[3][1]*movement

            #Right
            if self.heading == 'left':
                self.location[0] += self.movements[1][0]*movement
                self.location[1] += self.movements[1][1]*movement

            #print 'After movement'
            print 'Position: ' + str(self.location) + '. Orientation: ' + str(self.heading)
            #print 'Direction: ' + str(Direction) + '. Rotation: ' + str(rotation) + '. Movement ' + str(movement)

            # Exploration steps
            if self.is_exploring:
                self.steps_train += 1
            else:
                self.steps_real += 1
            ############################################################################################


        output = (rotation, movement)

        print 'Steps train: ' + str(self.steps_train) + '. Steps real: ' + str(self.steps_real)
        print 'Output ' + str(output)
        #print self.PathMap
        return output

    # Map the actual robot orientation and movement to make the right action
    def Convert_stage(self,Direction,Intensity):

        ######### Heading Up ##########
        if Direction == 0 and self.heading == 'up':
            movement = Intensity
            rotation = 0

        if Direction == 1 and self.heading == 'up':
            movement = Intensity
            rotation = -90

        if Direction == 2 and self.heading == 'up':
            movement = -Intensity
            rotation = 0

        if Direction == 3 and self.heading == 'up':
            movement = Intensity
            rotation = 90

        ############# Heading left #############
        if Direction == 0 and self.heading == 'left':
            movement = Intensity
            rotation = 90

        if Direction == 1 and self.heading == 'left':
            movement = Intensity
            rotation = 0

        if Direction == 2 and self.heading == 'left':
            movement = Intensity
            rotation = -90

        if Direction == 3 and self.heading == 'left':
            movement = -Intensity
            rotation = 0

        ############# Heading Down #############
        if Direction == 0 and self.heading == 'down':
            movement = -Intensity
            rotation = 0

        if Direction == 1 and self.heading == 'down':
            movement = Intensity
            rotation = 90

        if Direction == 2 and self.heading == 'down':
            movement = Intensity
            rotation = 0

        if Direction == 3 and self.heading == 'down':
            movement = Intensity
            rotation = -90

        ############# Heading Down #############
        if Direction == 0 and self.heading == 'right':
            movement = Intensity
            rotation = -90

        if Direction == 1 and self.heading == 'right':
            movement = -Intensity
            rotation = 0

        if Direction == 2 and self.heading == 'right':
            movement = Intensity
            rotation = 90

        if Direction == 3 and self.heading == 'right':
            movement = Intensity
            rotation = 0


        return movement,rotation

################ Random movement ##########
    def getNextDirectionRandom(self):
        Intensity = 1
        Next_direction = numpy.random.random_integers(0, 3, 1)[0] # Generate random movement
        Not_a_wall = False
        # Make sure there's not a wall in it
        while Not_a_wall == False:
            x2 = self.location[0] + self.movements[Next_direction][0]
            y2 = self.location[1] + self.movements[Next_direction][1]
            if x2 >= 0 and x2 < self.maze_dim and y2 >= 0 and y2 < self.maze_dim and self.barriers[Next_direction][self.Dimension_vector_mappingY[self.location[1]],self.location[0]] == 0 :
                Not_a_wall= True
            else:
                Next_direction = numpy.random.random_integers(0, 3, 1)[0]

        return Next_direction, Intensity
################ A* implementation ########
    def getNextDirectionAstar(self,goal):#search(grid, init, goal, cost, heuristic):


        Visited = np.zeros((self.maze_dim,self.maze_dim)) # If grid was visited
        Visited[self.Dimension_vector_mappingY[self.location[1]],self.location[0]] = 1

        Grid_values = np.zeros((self.maze_dim,self.maze_dim))-1 # Matrix to save values for exploration
        Grid_values[self.Dimension_vector_mappingY[self.location[1]],self.location[0]] = 0 #te
        te = 1
        action = np.zeros((self.maze_dim,self.maze_dim))

        h = self.heuristic[self.location[0], self.location[1]]
        To_explore = [[h, 0, self.location[0], self.location[1]]]# List of exploring variables

        is_goal = False  # if seach is completed


        # Mount map
        while not is_goal:# and not resign:
            if len(To_explore) == 0:

                return "How can that be return? But still... who knows"
            else:
                To_explore.sort(reverse=True) # Pop from the list and do this for avoid duplicating exploring

                Actual_position = To_explore.pop()
                x = Actual_position[2]
                y = Actual_position[3]
                g = Actual_position[1]
                h = Actual_position[0]



                if x == goal[0] and y == goal[1]: # If goal
                    is_goal = True
                    Grid_values[self.Dimension_vector_mappingY[y], x] = te
                else:
                    for idx in range(len(self.movements)): # Loop to see each movement
                        x2 = x + self.movements[idx][0]
                        y2 = y + self.movements[idx][1]

                        # All the conditions that would make the move invalid
                        if x2 >= 0 and x2 < self.maze_dim and y2 >= 0 and y2 < self.maze_dim:
                            if Visited[self.Dimension_vector_mappingY[y2]][x2] == 0 and \
                                            self.barriers[idx][self.Dimension_vector_mappingY[y],x] == 0 : # Each direction has its own barrier in the point of movement
                                g2 = g + 1
                                h2 = g2 + self.heuristic[x2,y2]
                                To_explore.append([ h2,g2, x2, y2])
                                Visited[self.Dimension_vector_mappingY[y2]][x2] = 1
                                Grid_values[self.Dimension_vector_mappingY[y2],x2] = te
                                te += 1

                                #print 'te: ' + str(te) + '. x2: ' + str(x2) + '. y2: ' + str(y2)
                                #print 'Saved position: ' +str(self.Dimension_vector_mappingY[y2]) + ' '  + str(x2)
                                #print 'Grid values' + str(Grid_values[self.Dimension_vector_mappingY[y2],x2])
        # Initialize variables to get next path
        #print 'Grid Values'
        #print str(Grid_values)

        # Get the next path
        #print 'te first ' +str(te)

        # Invert the movement, the actual movement to do is the inverse of the one done
        Movment_to_save = [2, 3, 0, 1]
        # Save repeated movements to get intensity [-3,3]
        Movement_memory = -1
        Intensity = 1

        actions = np.zeros((self.maze_dim,self.maze_dim))
        x2 = x
        y2 = y

        while te != 0: # Loop until get from goal to initial position
            x = x2
            y = y2
            Directions = [999, 999, 999, 999] # Start with big numbers
            Positions = [[0, 0], [0, 0], [0, 0], [0, 0]]
            for idx in range(len(self.movements)):

                x2 = x + self.movements[idx][0]
                y2 = y + self.movements[idx][1]

                #print 'Barrier: ' + str(self.barriers[0])
                if x2 >= 0 and x2 < self.maze_dim and y2 >= 0 and y2 < self.maze_dim and \
                                self.barriers[Movment_to_save[idx]][self.Dimension_vector_mappingY[y2],x2] == 0 \
                                and Grid_values[self.Dimension_vector_mappingY[y2], x2] > -1:
                    Directions[idx] = Grid_values[self.Dimension_vector_mappingY[y2]][x2]
                    Positions[idx] = [self.Dimension_vector_mappingY[y2], x2]

            Movement = np.argmin(np.array(Directions)) # Get the movement of the small number

            # Increase intensity for repeated movements
            if Movment_to_save[Movement] == Movement_memory:
                Intensity += 1
            else:
                Intensity = 1
            # Save in memory
            Movement_memory = Movment_to_save[Movement]


            # Change initial position to position with best movement
            x2 = x + self.movements[Movement][0]
            y2 = y + self.movements[Movement][1]
            te = Grid_values[Positions[Movement][0],Positions[Movement][1]]
            #print 'Grid Values'
            #print str(Grid_values)



            #print 'Directions: ' + str(Directions) + 'Location. ' + str(self.location) + 'Orientation: ' + str(self.heading)
            actions[Positions[Movement][0]][Positions[Movement][1]] = Movment_to_save[Movement]
            #print 'te: ' +str(te) + '. Action: ' + str(Movment_to_save[Movement])


            #print 'te ' +str(te)

            # See if back to the original position, and thus, get the right path
            if te == 0:
                # Save the taken movement
                self.PathMap[self.Dimension_vector_mappingY[y2], x2] = self.Representation[Movement]

                Direction = Movment_to_save[Movement]
                if Intensity > 3: # If higher than 3, go to 3
                    Intensity = 3
                break


        #print 'Actions' + str(actions)
        return Direction,Intensity