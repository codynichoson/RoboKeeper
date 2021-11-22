'''
Uses linear or quadratic fitting to predict the position of the ball at the goal line
'''
import numpy as np
import sympy as sym
import random
from sklearn import linear_model
import matplotlib.pyplot as plt
from sympy.core.symbol import Symbol
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
from sklearn.model_selection import train_test_split
from datetime import datetime

def traj_2D(x, y, x_goal):
    '''Predicts the y position of the ball when it crosses the goal line by fitting
    a line to the first few xy-positions of the ball

    Args:
        x (list): x coordinate training data
        y (list): y coordinate training data
        x_goal (float): position of the goal on the x-axis

    Returns:
        y_predict (float): predicted y postion of the ball when it crosses the goal
    '''
    # Compute linear regression model
    reg = linear_model.LinearRegression()
    reg.fit(x, y)
    intercept = reg.intercept_
    slope = reg.coef_

    # Now make the prediction
    y_predict = intercept + slope*x_goal

    return y_predict

def get_rosbag_data():
    '''
    Extracts position data stored in a rosbag file

    Only needed for testing
    '''
    bag = bagreader('subset_second.bag')
    data_rosbag = bag.message_by_topic('/Goalie_coord')
    data = pd.read_csv(data_rosbag)
    return data

def main():
    '''
    Used for testing
    '''
    data = get_rosbag_data().to_numpy()
    print(datetime.now())
    x = data[:, 1]
    y = data[:, 2]

    n = 10
    x_train = np.array([x[0:n]]).T
    y_train = y[0:n]

    x_goal = -1.27  # As of Nov 21, 8pm
    y_predict = traj_2D(x_train, y_train, x_goal)
    print(datetime.now())

    print(y_predict)

if __name__ == '__main__':
    main()
