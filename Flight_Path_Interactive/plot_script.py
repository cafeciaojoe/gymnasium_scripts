import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
'''
This script reads all the .csv files located in a folder and creates
3D plots with all the coordinates of all collected points.

You need to install the packages listed above and to do that, you
might need to create a python virtual environment.

The format of the .csv files is:  Year-Month-Day_Hour-Minute-Second
'''


def simple_plot(folder_path='Flight_Path_Interactive/data_files'):  # This should point to the CSVs folder

    # Get all CSV files in the folder
    csv_files = glob.glob(os.path.join(folder_path, '*.csv'))

    for file in csv_files:
        df = pd.read_csv(file)

        x = df['x'].to_numpy()
        y = df['y'].to_numpy()
        z = df['z'].to_numpy()

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        # Scatter plot
        ax.scatter(x, y, z, color='blue', s=50)

        # Add numbered labels
        for i in range(len(x)):
            ax.text(x[i], y[i], z[i], f'{i}', fontsize=10, color='red')

        # Axis labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Adjust limits to include 0
        ax.set_xlim(min(0, min(x)), max(0, max(x)))
        ax.set_ylim(min(0, min(y)), max(0, max(y)))
        ax.set_zlim(min(0, min(z)), max(0, max(z)))
        ax.set_box_aspect([1, 1, 1])  # Equal aspect ratio

        # Title with filename
        plt.title(f'3D Setpoints: {os.path.basename(file)}')
        plt.show()


if __name__ == "__main__":
    simple_plot()