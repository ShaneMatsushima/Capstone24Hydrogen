import pandas as pd
import numpy as np

data_raw = pd.read_csv('H2_simData.csv')

TOTAL_TIME = 8 * 60 * 60 # Hours to seconds
data_raw = np.array(data_raw)


if __name__ == '__main__':
    row,col = data_raw.shape
    CELL_TIME = TOTAL_TIME / row

    print("Cell Time: ",CELL_TIME)
