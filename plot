import numpy as np
import matplotlib.pyplot as plt

def plot_mean_and_std(values):
    mean_values = np.mean(values)
    std_values = np.std(values)

    y = np.linspace(0, 1, len(values))
    
    print(mean_values)
    print(std_values)
    
    plt.hist(values, width=0.1)
    #g.set_xticks(len(values),labels=values)#plt.axis(xlim=(min(values), max(values)))
    plt.axvline(
        mean_values, 
        color = 'g',
        linestyle = '--', 
        linewidth = 3
    )
    
    plt.axvline(
        mean_values + std_values, 
        color = 'r',
        linestyle = '--', 
        linewidth = 3
    )
    plt.axvline(
        mean_values - std_values, 
        color = 'r',
        linestyle = '--', 
        linewidth = 3
    )
    plt.show()





values = np.array([13,24,12,10,19,8,7,9,31])

plot_mean_and_std(values)