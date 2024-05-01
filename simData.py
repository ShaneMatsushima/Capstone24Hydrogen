import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

data_raw = pd.read_csv('Dummy Data.csv')

TOTAL_TIME = 8 * 60 * 60 # Hours to seconds
data_raw = np.array(data_raw)

def pickFM(data:int) -> int:
    if data < 0.54:
        return -1
    elif data >= 0.54 and data <= 9:
        return 1
    elif data > 9 and data <= 24:
        return 2
    else:
        return -1

if __name__ == '__main__':
    row,col = data_raw.shape
    CELL_TIME = TOTAL_TIME / row

    time_axis = []
    flow_tmp = []
    data = pd.DataFrame()

    for i in range(row):
        time_axis.append(i*CELL_TIME)
        flow_tmp.append(data_raw[i][0])
    data['Flow'] = flow_tmp

    print("<<<ADDING FLOW>>>")

    tmplst = []

    for i in range(data['Flow'].size):
        tmplst.append(pickFM(data['Flow'][i]))
    data['FlowChoice'] = tmplst

    print("<<<ADDING FLOW CHOICE>>>")

    flow_data = np.array(data['Flow'])
    flow_choice_data = np.array(data['FlowChoice'], dtype=int)

    print("<<<PLOTTING DATA>>>")

    plt.plot(time_axis, flow_data, label='Simulated Flow')
   # plt.plot(time_axis, flow_choice_data, color = 'green', label='Flow Meter Choice')

    plt.xlabel('Time (s)')
    plt.ylabel('Flow')
    plt.title('Simulation Data')

    plt.show()
