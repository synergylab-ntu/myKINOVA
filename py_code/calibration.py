import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score
import math

def euler_to_rot(df, index=-1):
    rot_mat =  [[None for x in range(3)] for y in range(3)]
    
    if index >= 0:
        rot_mat[0][0] = ( np.cos(df['theta_z'][index]) * np.cos(df['theta_y'][index]) )
        rot_mat[0][1] = ( (np.cos(df['theta_z'][index]) * np.sin(df['theta_y'][index]) * np.sin(df['theta_x'][index])) - (np.sin(df['theta_z'][index]) * np.cos(df['theta_x'][index])) )
        rot_mat[0][2] = ( (np.cos(df['theta_z'][index]) * np.sin(df['theta_y'][index]) * np.cos(df['theta_x'][index])) + (np.sin(df['theta_z'][index]) * np.sin(df['theta_x'][index])) )

        rot_mat[1][0] = ( np.sin(df['theta_z'][index]) * np.cos(df['theta_y'][index]) )
        rot_mat[1][1] = ( (np.sin(df['theta_z'][index]) * np.sin(df['theta_y'][index]) * np.sin(df['theta_x'][index])) + (np.cos(df['theta_z'][index]) * np.cos(df['theta_x'][index])) )
        rot_mat[1][2] = ( (np.sin(df['theta_z'][index]) * np.sin(df['theta_y'][index]) * np.cos(df['theta_x'][index])) - (np.cos(df['theta_z'][index]) * np.sin(df['theta_x'][index])) )

        rot_mat[2][0] = ( -np.sin(df['theta_y'][index]) )
        rot_mat[2][1] = ( np.cos(df['theta_y'][index]) * np.sin(df['theta_x'][index]) )
        rot_mat[2][2] = ( np.cos(df['theta_y'][index]) * np.cos(df['theta_x'][index]) )
    else:
        rot_mat[0][0] = ( np.cos(df['theta_z']) * np.cos(df['theta_y']) )
        rot_mat[0][1] = ( (np.cos(df['theta_z']) * np.sin(df['theta_y']) * np.sin(df['theta_x'])) - (np.sin(df['theta_z']) * np.cos(df['theta_x'])) )
        rot_mat[0][2] = ( (np.cos(df['theta_z']) * np.sin(df['theta_y']) * np.cos(df['theta_x'])) + (np.sin(df['theta_z']) * np.sin(df['theta_x'])) )

        rot_mat[1][0] = ( np.sin(df['theta_z']) * np.cos(df['theta_y']) )
        rot_mat[1][1] = ( (np.sin(df['theta_z']) * np.sin(df['theta_y']) * np.sin(df['theta_x'])) + (np.cos(df['theta_z']) * np.cos(df['theta_x'])) )
        rot_mat[1][2] = ( (np.sin(df['theta_z']) * np.sin(df['theta_y']) * np.cos(df['theta_x'])) - (np.cos(df['theta_z']) * np.sin(df['theta_x'])) )

        rot_mat[2][0] = ( -np.sin(df['theta_y']) )
        rot_mat[2][1] = ( np.cos(df['theta_y']) * np.sin(df['theta_x']) )
        rot_mat[2][2] = ( np.cos(df['theta_y']) * np.cos(df['theta_x']) )

    return rot_mat

def main():
    df = pd.read_csv("D:/KINOVA/api_cpp/examples/py_code/Calibration/1kHz_calibration_log_file_2021_3_15_227142.csv")
    
    fig, ax = plt.subplots(num=0)
    fig.suptitle("Damping")
    ax.plot(df['qdot_1'], df['j_t1'])
    plt.grid(True)
    plt.show()
    # print(df['j_t0'])


if __name__ == "__main__":
    main()