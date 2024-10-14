import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-0.5, 1.5, 2000)

y1 = np.where((x < 0) | (x > 1), 0, 1 / ( 1 + np.exp(-10 * (x - 0.5))))

y2 = np.where((x < 0) | (x > 1), 0, 1 / ( 1 + np.exp(-10 * (1 - x - 0.5))))

plt.rcParams['font.size'] = 16
fig, axs = plt.subplots(2, 1, figsize=(10, 8))  # 2 Zeilen, 2 Spalten

# Schritt 2: Plot in jedem Subplot
# 1. Subplot

axs[0].plot(x, y1, label='sig1 = 1 / (1 + exp(-10*(x - 0.5))', color='blue')
axs[0].set_title('First sigmoid-right side of the segment')
axs[0].set_xlabel('x')
axs[0].set_ylabel('f(x)')
axs[0].legend()
axs[0].grid()

# 2. Subplot
axs[1].plot(x, y2, label='sig2 = 1 / (1 + exp(-10*(1 - x - 0.5))', color='orange')
axs[1].set_title('Second sigmoid-left side of the segment')
axs[1].set_xlabel('x')
axs[1].set_ylabel('f(x)')
axs[1].legend()
axs[1].grid()

fig2, axs2 = plt.subplots(2, 1, figsize=(10, 8))  # 2 Zeilen, 2 Spalten

b_low = 0.5
mid = 0.6 
b_high = 0.65
b_1 = 1 / (mid - b_low)
b_2 = 1 / (b_high - mid)

x2 = np.linspace(0, 1, 2000)
y2_1 = np.where((x2 < b_low) | (x2 > mid), 0, 1 / ( 1 + np.exp(-10 * (b_1*(x2 - b_low) - 0.5))))

y2_2 = np.where((x2 < mid) | (x2 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x2 - mid) - 0.5))))

#3. Subplot
axs2[0].plot(x2, y2_1, label='sigmoid 1', color='green')
axs2[0].plot(x2, y2_2, label='sigmoid 2', color='blue', alpha=0.5)
axs2[0].set_title('Combination of both sigmoids')
axs2[0].set_xlabel('x')
axs2[0].set_ylabel('f(x)')
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
axs2[0].set_xticks(ticks)
axs2[0].legend()
axs2[0].grid()


# 4. Subplot
b_low = 0.5
mid = 0.6 
b_high = 0.65
b_1 = 1 / (mid - b_low)
b_2 = 1 / (b_high - mid)

x3 = np.linspace(0, mid, 2000)
x4 = np.linspace(mid, 1 , 2000)
y3_1 = np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

y3_2 = np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))

axs2[1].plot(x3, y3_1, label='sigmoid 1', color='green')
axs2[1].plot(x4, y3_2, label='sigmoid 2', color='blue')
axs2[1].set_title('Combination of both sigmoids - cleaned')
axs2[1].set_xlabel('x')
axs2[1].set_ylabel('f(x)')
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
axs2[1].set_xticks(ticks)
#axs2.tick_params(axis='both', labelsize=12)
axs2[1].legend()
axs2[1].grid()
#lt.rcParams['font.size'] = 22
plt.show()