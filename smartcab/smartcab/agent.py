import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator
import numpy as np
import matplotlib.pyplot as plt
class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
        self.states = []
        self.QTable = []
        self.alpha = 0.8
        self.discount_factor = 0.999
        self.time = 0
        self.totalreward = 0
        self.penalities = 0
        self.general_steps = 0
        self.wrong_moves_per_run = []


    def reset(self, destination=None):
        # TODO: Prepare for a new trip; reset any variables here, if required

        self.planner.route_to(destination)
        #self.time = 0
        self.general_steps += 1
        self.totalreward = 0


        # Save penalities
        self.wrong_moves_per_run.append(self.penalities)
        self.penalities = 0

    # Function to get the actual state and save it in the states variable
    def get_actual_State(self,Stage_temp):
        if Stage_temp in self.states:
            # Find the index
            t = 0
            Actual_state = 0
            for idx in np.arange(0,len(self.states)):
                Is_index = Stage_temp == self.states[t]
                if Is_index:
                    Actual_state = t
                    pass


                t += 1
            pass
        else:
             self.states.append(Stage_temp)
             Actual_state = len(self.states) - 1
        return Actual_state

    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)

        # TODO: Update state

        # Create the state
        Stage_temp = [inputs.get('light'),inputs.get('oncoming'),inputs.get('left')#inputs.get('Right')
            ,self.next_waypoint]

        Actual_state = self.get_actual_State(Stage_temp)
        self.state = Actual_state

        # TODO: Select action according to your policy

        # Create/Adjust table
        if Actual_state > len(self.QTable)-1:
            #self.QTable = np.hstack((self.QTable,(np.array([0,0,0,0]))))
            self.QTable.append([0,0,0,0])

        # Choose action
        valid_actions = [None, 'forward', 'left', 'right']
        epsilon = 0.9
        if random.random()*0.9999**self.general_steps > epsilon: # decrease random exploration while learning
            action = valid_actions[random.randrange(0, 4, 1)]
        else:
            action = valid_actions[np.argmax(self.QTable[Actual_state][:])]
        #action = valid_actions[random.randrange(0, 4, 1)] #UNDO THIS FOR RANDOM AGENT

        # Rewards
        reward = self.env.act(self, action)
        self.totalreward += reward

        # Increase penality if agent makes a bad move
        if reward == -1:
            self.penalities += 1
            #print "Negative reward: inputs = {}, action = {}, reward = {}, waypoint {}".format(inputs, action, reward,
             #                                                                              self.next_waypoint)

        # Convert the actions to numbers
        if action == None:
            action = 0
        if action == 'forward':
            action = 1
        if action == 'left':
            action = 2
        if action == 'right':
            action = 3

        # Get New State
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        Stage_new= [inputs.get('light'), inputs.get('oncoming'), inputs.get('left'),# inputs.get('Right'),
                      self.next_waypoint]

        New_state = self.get_actual_State(Stage_new)

        # If it is a new state, put it in the table
        if New_state > len(self.QTable)-1:
            self.QTable.append([0,0,0,0])

        # TODO: Learn policy based on state, action, reward

        Future_reward = np.max(self.QTable[New_state][:])


        self.QTable[Actual_state][action] =self.QTable[Actual_state][action] + self.alpha*(reward + self.discount_factor**self.time * Future_reward - self.QTable[Actual_state][action])


        # Increase time
        self.time += 1

        # Print total reward
        #print 'The total reward was: ' + str(self.totalreward)


        #print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]

        # See the best action from each states based on the table
        #t = 0
        #if self.general_steps == 100:
            #for state in self.states:

                #print 'State ' +str(state) +'. Best action: ' + str((str(np.argmax(self.QTable[t][:]))))
                #t += 1


        #print 'Number of states: ' + str( len(self.QTable))

        # Print penality
        #print 'The total number of penalities was: ' +str(self.penalities)

def run():
    """Run the agent for a finite number of trials."""

    Number_repetitions = 1
    Rate = np.zeros((Number_repetitions,1))
    Rate20 = np.zeros((Number_repetitions,1))
    Penalty20 = np.zeros((Number_repetitions, 1))

    # Loop to average
    for idx in np.arange(0,Number_repetitions,1):
    # Set up environment and agent
        e = Environment()  # create environment (also adds some dummy traffic)
        a = e.create_agent(LearningAgent)  # create agent
        e.set_primary_agent(a, enforce_deadline=True)  # specify agent to track
        # NOTE: You can set enforce_deadline=False while debugging to allow longer trials

        # Now simulate it
        sim = Simulator(e, update_delay=0.0000001, display=True)  # create simulator (uses pygame when display=True, if available)
        # NOTE: To speed up simulation, reduce update_delay and/or set display=False

        sim.run(n_trials=100)  # run for a specified number of trials
        # NOTE: To quit midway, press Esc or close pygame window, or hit Ctrl+C on the command-line

        # I've edited the enviroment variable to do the plot creating an completions array
        completions = np.array(e.completions)
        rate = float(completions.sum())/float((len(completions)))
        rate20 = float(completions[-20:].sum())/20

        Rate[idx] = rate
        Rate20[idx] = rate20

        Wrong = np.array(a.wrong_moves_per_run[-20:]).mean()
        Penalty20[idx] = Wrong

        plt.scatter(np.arange(0,len(completions)),completions)
        plt.plot(Wrong)
        plt.xlabel('Trial')
        plt.ylabel('1 = Get in the destination, 0 = did not get')
        plt.title('Reiforcement learning progress')
        plt.legend(['Rate of completion: ' + str(rate) + '. Rate last 20: ' + str(rate20) + '.Mean penalty last 20: ' + str(Wrong)])
        plt.show()

    #print 'Accuracy: ' + str(Rate) + '. Mean: ' + str(np.mean(Rate))
    #print 'Mean 20: ' + str(np.mean(Rate20))#'Accuracy 20: ' + str(Rate20) + '. Mean 20: ' + str(np.mean(Rate20))
    #print 'Mean_penalty: ' + str(np.mean(Penalty20))

    # Print state table with actions
    #t = 0
    #for state in a.states:
        #print 'State ' + str(state) + '. Best action: ' + str((str(np.argmax(a.QTable[t][:]))))
        #t += 1
if __name__ == '__main__':
    run()
