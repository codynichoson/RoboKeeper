'''
Uses linear or quadratic fitting to predict the position of the ball at the goal line
'''
import numpy as np
import random
from sklearn import linear_model
import matplotlib.pyplot as plt

def traj_2D(X, y, n):
    '''Prediction using linear regression

    Args:
        X:
        y:
        n (int): number of data points used for training linear regression model

    Returns:
        prediction:
    '''
    reg = linear_model.LinearRegression()
    reg.fit(X[:n], y[:n])
    prediction = reg.predict(X)
    return prediction

def main():
    # Get some random-ish data
    linear_data = range(50)
    messy_data = []
    for point in linear_data:
        messy_data.append(point + random.uniform(0,4))

    X = np.array([messy_data]).T
    y = np.array(linear_data)
    n = 50  # number of points used in fitting
    prediction = traj_2D(X, y, n)

    plt.scatter(X, y)
    plt.plot(X, prediction, color='blue')
    plt.show()

if __name__ == '__main__':
    main()
