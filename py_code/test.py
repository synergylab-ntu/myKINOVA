import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
import math

def plotting(df):
    plt.figure()
    plt.subplot(211)
    plt.plot(df[1], df[2])

    plt.subplot(212)
    plt.plot(df[0], df[2])
    plt.show()

def calc_damping(df):
    x = np.array(df[1]).reshape((-1,1)) #X = velocity
    y = np.array(df[2]) #Y = torque
    x_pred = np.linspace(-0.5, 0.5, df.shape[0])

    model = LinearRegression(fit_intercept=False).fit(x,y)
    y_pred = model.predict(x_pred.reshape(-1,1))

    print("Damping: ", model.coef_[0])
    print("Intercept: ", model.intercept_)
    print("Mean Squared Error: ", mean_squared_error(y, y_pred)) #MSE
    print("Coefficient of Determiniation: ", r2_score(y, y_pred)) #R^2 score

    plt.figure()
    plt.plot(df[1], df[2])
    plt.plot(x_pred.reshape(-1,1),y_pred)
    plt.show()

def main():
    df = pd.read_csv("D:/KINOVA/api_cpp/examples/logs/log_output_6.csv", header=None)
    for i in range(df[1].size):
        df[1][i] = math.radians(df[1][i])
    #plotting(df)
    calc_damping(df)
    

if __name__ == "__main__":
    main()