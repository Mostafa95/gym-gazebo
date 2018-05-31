#!/usr/bin/env python

'''
Based on:
https://github.com/vmayoral/basic_reinforcement_learning
https://gist.github.com/wingedsheep/4199594b02138dd427c22a540d6d6b8d
'''

import gym
import gym_gazebo
import time
from distutils.dir_util import copy_tree
import os
import json
import random
import numpy as np
from keras.models import Sequential, load_model
from keras import optimizers
from keras.layers.core import Dense, Dropout, Activation
from keras.layers.normalization import BatchNormalization
from keras.layers.advanced_activations import LeakyReLU
from keras.regularizers import l2
import memory
import rospy
from nav_msgs.msg import OccupancyGrid

class DeepQ:
    """
    DQN abstraction.

    As a quick reminder:
        traditional Q-learning:
            Q(s, a) += alpha * (reward(s,a) + gamma * max(Q(s') - Q(s,a))
        DQN:
            target = reward(s,a) + gamma * max(Q(s')

    """
    def __init__(self, inputs, outputs, memorySize, discountFactor, learningRate, learnStart):
        """
        Parameters:
            - inputs: input size
            - outputs: output size
            - memorySize: size of the memory that will store each state
            - discountFactor: the discount factor (gamma)
            - learningRate: learning rate
            - learnStart: steps to happen before for learning. Set to 128
        """
        self.input_size = inputs
        self.output_size = outputs
        self.memory = memory.Memory(memorySize)
        self.discountFactor = discountFactor
        self.learnStart = learnStart
        self.learningRate = learningRate

    def initNetworks(self, hiddenLayers):
        model = self.createModel(self.input_size, self.output_size, hiddenLayers, "relu", self.learningRate)
        self.model = model

        targetModel = self.createModel(self.input_size, self.output_size, hiddenLayers, "relu", self.learningRate)
        self.targetModel = targetModel

    def createRegularizedModel(self, inputs, outputs, hiddenLayers, activationType, learningRate):
        bias = True
        dropout = 0
        regularizationFactor = 0.01
        model = Sequential()
        if len(hiddenLayers) == 0:
            model.add(Dense(self.output_size, input_shape=(self.input_size,), init='lecun_uniform', bias=bias))
            model.add(Activation("linear"))
        else :
            if regularizationFactor > 0:
                model.add(Dense(hiddenLayers[0], input_shape=(self.input_size,), init='lecun_uniform', W_regularizer=l2(regularizationFactor),  bias=bias))
            else:
                model.add(Dense(hiddenLayers[0], input_shape=(self.input_size,), init='lecun_uniform', bias=bias))

            if (activationType == "LeakyReLU") :
                model.add(LeakyReLU(alpha=0.01))
            else :
                model.add(Activation(activationType))

            for index in range(1, len(hiddenLayers)):
                layerSize = hiddenLayers[index]
                if regularizationFactor > 0:
                    model.add(Dense(layerSize, init='lecun_uniform', W_regularizer=l2(regularizationFactor), bias=bias))
                else:
                    model.add(Dense(layerSize, init='lecun_uniform', bias=bias))
                if (activationType == "LeakyReLU") :
                    model.add(LeakyReLU(alpha=0.01))
                else :
                    model.add(Activation(activationType))
                if dropout > 0:
                    model.add(Dropout(dropout))
            model.add(Dense(self.output_size, init='lecun_uniform', bias=bias))
            model.add(Activation("linear"))
        optimizer = optimizers.RMSprop(lr=learningRate, rho=0.9, epsilon=1e-06)
        model.compile(loss="mse", optimizer=optimizer)
        model.summary()
        return model

    def createModel(self, inputs, outputs, hiddenLayers, activationType, learningRate):
        model = Sequential()
        if len(hiddenLayers) == 0:
            model.add(Dense(self.output_size, input_shape=(self.input_size,), init='lecun_uniform'))
            model.add(Activation("linear"))
        else :
            model.add(Dense(hiddenLayers[0], input_shape=(self.input_size,), init='lecun_uniform'))
            if (activationType == "LeakyReLU") :
                model.add(LeakyReLU(alpha=0.01))
            else :
                model.add(Activation(activationType))

            for index in range(1, len(hiddenLayers)):
                # print("adding layer "+str(index))
                layerSize = hiddenLayers[index]
                model.add(Dense(layerSize, init='lecun_uniform'))
                if (activationType == "LeakyReLU") :
                    model.add(LeakyReLU(alpha=0.01))
                else :
                    model.add(Activation(activationType))
            model.add(Dense(self.output_size, init='lecun_uniform'))
            model.add(Activation("linear"))
        optimizer = optimizers.RMSprop(lr=learningRate, rho=0.9, epsilon=1e-06)
        model.compile(loss="mse", optimizer=optimizer)
        model.summary()
        return model

    def printNetwork(self):
        i = 0
        for layer in self.model.layers:
            weights = layer.get_weights()
            print("layer ",i,": ",weights)
            i += 1


    def backupNetwork(self, model, backup):
        weightMatrix = []
        for layer in model.layers:
            weights = layer.get_weights()
            weightMatrix.append(weights)
        i = 0
        for layer in backup.layers:
            weights = weightMatrix[i]
            layer.set_weights(weights)
            i += 1

    def updateTargetNetwork(self):
        self.backupNetwork(self.model, self.targetModel)

    # predict Q values for all the actions
    def getQValues(self, state):
        #print 'BBBBBBBBBBSTate',state,'\n',type(state)
        # state = np.asarray(state[0])
        #print 'AAAABBBBBBBBBBSTate',state,'\n',type(state)
       # print 'AAAAAAAAAASTate',state.reshape(1,len(state)),'\n'
        
        predicted = self.model.predict(state.reshape(1,len(state)))#state.reshape(1,len(state))
        #print 'Predicted Actions : ',predicted
        return predicted[0]

    def getTargetQValues(self, state):
        #predicted = self.targetModel.predict(state.reshape(1,len(state)))
        predicted = self.targetModel.predict(state.reshape(1,len(state)))

        return predicted[0]

    def getMaxQ(self, qValues):
        return np.max(qValues)

    def getMaxIndex(self, qValues):
        return np.argmax(qValues)

    # calculate the target function
    def calculateTarget(self, qValuesNewState, reward, isFinal):
        """
        target = reward(s,a) + gamma * max(Q(s')
        """
        if isFinal:
            return reward
        else :
            return reward + self.discountFactor * self.getMaxQ(qValuesNewState)

    # select the action with the highest Q value
    def selectAction(self, qValues, explorationRate):
        rand = random.random()
        if rand < explorationRate :
            #print 'Action Selected Randomly'
            action = np.random.randint(0, self.output_size)
        else :
            #print 'Max Q Action Selected'
            action = self.getMaxIndex(qValues)
        return action


    def selectAction_ForTesting(self, qValues):    
        action = self.getMaxIndex(qValues)
        return action

    def selectActionByProbability(self, qValues, bias):
        qValueSum = 0
        shiftBy = 0
        for value in qValues:
            if value + shiftBy < 0:
                shiftBy = - (value + shiftBy)
        shiftBy += 1e-06

        for value in qValues:
            qValueSum += (value + shiftBy) ** bias

        probabilitySum = 0
        qValueProbabilities = []
        for value in qValues:
            probability = ((value + shiftBy) ** bias) / float(qValueSum)
            qValueProbabilities.append(probability + probabilitySum)
            probabilitySum += probability
        qValueProbabilities[len(qValueProbabilities) - 1] = 1

        rand = random.random()
        i = 0
        for value in qValueProbabilities:
            if (rand <= value):
                return i
            i += 1

    def addMemory(self, state, action, reward, newState, isFinal):
        self.memory.addMemory(state, action, reward, newState, isFinal)

    def learnOnLastState(self):
        if self.memory.getCurrentSize() >= 1:
            return self.memory.getMemory(self.memory.getCurrentSize() - 1)

    def learnOnMiniBatch(self, miniBatchSize, useTargetNetwork=True):
        # Do not learn until we've got self.learnStart samples
        if self.memory.getCurrentSize() > self.learnStart:
            # learn in batches of 128
            miniBatch = self.memory.getMiniBatch(miniBatchSize)
            X_batch = np.empty((0,self.input_size), dtype = np.float64)
            Y_batch = np.empty((0,self.output_size), dtype = np.float64)
            for sample in miniBatch:
                isFinal = sample['isFinal']
                state = sample['state']
                action = sample['action']
                reward = sample['reward']
                newState = sample['newState']
                
                #print 'NNNNNNNNNNWsTate',newState,'\n',type(newState)
                state = np.asarray(state)
                newState = np.asarray(newState)
                
                qValues = self.getQValues(state)
                if useTargetNetwork:
                    qValuesNewState = self.getTargetQValues(newState)
                else :
                    qValuesNewState = self.getQValues(newState)
                targetValue = self.calculateTarget(qValuesNewState, reward, isFinal)

                X_batch = np.append(X_batch, np.array([state.copy()]), axis=0)
                Y_sample = qValues.copy()
                Y_sample[action] = targetValue
                Y_batch = np.append(Y_batch, np.array([Y_sample]), axis=0)
                if isFinal:
                    X_batch = np.append(X_batch, np.array([newState.copy()]), axis=0)
                    Y_batch = np.append(Y_batch, np.array([[reward]*self.output_size]), axis=0)
            self.model.fit(X_batch, Y_batch, batch_size = len(miniBatch), nb_epoch=1, verbose = 0)

    def saveModel(self, path):
        self.model.save(path)
    
    def loadWeights(self, path):
        self.model.set_weights(load_model(path).get_weights())

def detect_monitor_files(training_dir):
    return [os.path.join(training_dir, f) for f in os.listdir(training_dir) if f.startswith('openaigym')]

def clear_monitor_files(training_dir):
    files = detect_monitor_files(training_dir)
    if len(files) == 0:
        return
    for file in files:
        print(file)
        os.unlink(file)


def callback(data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        print len(data.data)

def initMapSubscriber():
    #rospy.init_node('Maplistener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid,  callback) 
    #rospy.spin()

def Get_Observ(TurtleBot_path, Turtle_Num):
    params_json = str(TurtleBot_path) + str(Turtle_Num) + '.json'
    observation =  []
    with open(params_json) as outfile:
        d = json.load(outfile)
        observation = d.get('obervation')
    
    return observation

def Get_All(TurtleBot_path, Turtle_Num):
    params_json = str(TurtleBot_path) + str(Turtle_Num) + '.json'

    with open(params_json) as outfile:
        d = json.load(outfile)
        observation = d.get('obervation')
        reward = d.get('reward')
        done = d.get('done')
        info = d.get('info')

    return observation,reward,done,info

def Write_Action(TurtleBot_path, Turtle_Num,Action):
    ob = 0
    re = 0
    do = 0
    info = 0 
    parameter_keys = ['obervation','reward','done','info','action']
    parameter_values = [ob, re, do, info, Action] 
    parameter_dictionary = dict(zip(parameter_keys, parameter_values))
    fileName =  str(TurtleBot_path) + str(Turtle_Num) + '.json'
    with open(fileName, 'w') as outfile:
        json.dump(parameter_dictionary, outfile)


if __name__ == '__main__':

    env = gym.make('GazeboMazeTurtlebotLidar-v0') #GazeboCircuit2TurtlebotLidar-v0 GazeboCircuit2TurtlebotLidar-v0
   
    outdir = '/home/mostafa/GP_Training/gazebo_gym_experiments/'

    weights_path = '/home/mostafa/GP_Training/turtle_c2_dqn_ep1500.h5'
    monitor_path = '/home/mostafa/GP_Training/turtle_c2_dqn_ep1500'
    params_json  = '/home/mostafa/GP_Training/turtle_c2_dqn_ep1500.json'

    TurtleBot_info_path = '/home/mostafa/GP_Training/TurtleBot/'
    
    with open(params_json) as outfile:
        d = json.load(outfile)
        epochs = d.get('epochs')
        steps = d.get('steps')
        updateTargetNetwork = d.get('updateTargetNetwork')
        explorationRate = d.get('explorationRate')
        minibatch_size = d.get('minibatch_size')
        learnStart = d.get('learnStart')
        learningRate = d.get('learningRate')
        discountFactor = d.get('discountFactor')
        memorySize = d.get('memorySize')
        network_inputs = d.get('network_inputs')
        network_outputs = d.get('network_outputs')
        network_structure = d.get('network_structure')
        current_epoch = d.get('current_epoch')

    deepQ = DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
    deepQ.initNetworks(network_structure)
    deepQ.loadWeights(weights_path)

    start_time = time.time()
    f=True
    
    for epoch in xrange(current_epoch+1, epochs+1, 1):
        observation1 = env.reset()
        observation2 = Get_Observ(TurtleBot_info_path,2)
        observation3 = Get_Observ(TurtleBot_info_path,3)
        
        #print "Observation",observation1
        cumulated_reward1 = 0
        cumulated_reward2 = 0
        cumulated_reward3 = 0
        # number of timesteps
        t=1
        time.sleep(8)
        while( t > 0 ):
            
            # if f == True:
            #     time.sleep(20)
            #     f = False
            # else :
            #     time.sleep(0.5)    
            if type(observation1[1]) == bool :
                observation1 = observation1[0]
            observation1 = np.asarray(observation1)
            
            if type(observation2[1]) == bool :
                observation2 = observation2[0]
            observation2 = np.asarray(observation2)
            
            if type(observation3[1]) == bool :
                observation3 = observation3[0]
            observation3 = np.asarray(observation3)
            
            
            qValues1 = deepQ.getQValues(observation1)
            qValues2 = deepQ.getQValues(observation2)
            qValues3 = deepQ.getQValues(observation3)

            action1 = deepQ.selectAction_ForTesting(qValues1)
            action2 = deepQ.selectAction_ForTesting(qValues2)
            action3 = deepQ.selectAction_ForTesting(qValues3)

            Write_Action(TurtleBot_info_path, 2, action2)
            Write_Action(TurtleBot_info_path, 3, action3)

            newObservation1, reward1, done1, info1 = env.step(action1)
            newObservation2, reward2, done2, info2 = Get_All(TurtleBot_info_path, 2)
            newObservation3, reward3, done3, info3 = Get_All(TurtleBot_info_path, 3)

            cumulated_reward1 += reward1
            cumulated_reward2 += reward2
            cumulated_reward3 += reward3
            

            observation1 = newObservation1
            observation2 = newObservation2
            observation3 = newObservation3

    env.close()
