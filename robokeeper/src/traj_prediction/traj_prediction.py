'''
Uses linear or quadratic fitting to predict the position of the ball at the goal line
'''
import numpy as np
from sklearn import linear_model
import matplotlib.pyplot as plt
from bagpy import bagreader
import pandas as pd

def traj_linear_reg(x_train, y_train, x_goal):
    '''Predicts the y position of the ball when it crosses the goal line by fitting
    a line to the first few xy-positions of the ball

    Args:
        x_train (list): x coordinate training data
        y_train (list): y coordinate training data
        x_goal (float): position of the goal on the x-axis

    Returns:
        y_predict (float): predicted y postion of the ball when it crosses the goal
    '''
    # Format the input data for the linear regression model function
    x_train = np.array([x_train]).T
    y_train = np.array(y_train)

    # Compute linear regression model
    reg = linear_model.LinearRegression()
    reg.fit(x_train, y_train)
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
    x_data = data[:, 1]
    y_data = data[:, 2]

    n = 10
    x_train = x_data[0:n]
    y_train = y_data[0:n]

    x_goal = -1.27  # As of Nov 21, 8pm
    y_predict = traj_linear_reg(x_train, y_train, x_goal)

    print(y_predict)

if __name__ == '__main__':
    main()
