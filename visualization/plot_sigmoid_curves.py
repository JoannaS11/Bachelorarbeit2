import numpy as np
import matplotlib.pyplot as plt


plt.rcParams['font.size'] = 40
b_low = 0.5
mid = 0.6 
b_high = 0.65
b_1 = 1 / (mid - b_low)
b_2 = 1 / (b_high - mid)
b_low1, c_mid1, b_high1 = [0, 0.1, 0.15]
b_low2, c_mid2, b_high1 = [0.15, 0.2, 0.25]
b_low3, c_mid3, b_high1 = [0.25, 0.35, 0.4]
b_low4, c_mid4, b_high1 = [0.4, 0.5, 0.6]
b_low5, c_mid5, b_high1 = [0.6, 0.75, 0.8]
b_low6, c_mid6, b_high1 = [0.8, 0.85, 1]


low = np.array((0,0.15, 0.25, 0.4, 0.6, 0.8))
mid_ = np.array((0.1, 0.2, 0.35, 0.5, 0.75, 0.85))
high = np.array((0.15,0.25, 0.4,0.6, 0.8, 1))

z = -1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    y1 = z * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

    y2 = z * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = -1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if z == 1:
        y1 = z * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = z * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)

    y1 = z * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

    y2 = z * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))


    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i == np.shape(low)[0]//2 or i == np.shape(low)[0]//2 -1:
        y1 = z * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = z * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
plt.xlabel('x')
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i == np.shape(low)[0]//2 or i == np.shape(low)[0]//2 -1:
        y1 = -1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = -1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    elif (i == np.shape(low)[0]//2-2 or i == np.shape(low)[0]//2 +1):
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i == np.shape(low)[0]//2-2 or i == np.shape(low)[0]//2 +1:
        y1 = -1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = -1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    elif (i == np.shape(low)[0]//2-3 or i == np.shape(low)[0]//2 +2):
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i == 0:
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i ==1:
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i ==2:
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i ==3:
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i <=3:
        y1 = -1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = -1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    elif i ==4:
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

z = 1
for i in range(np.shape(low)[0]):
    
    b_low = low[i]
    mid = mid_[i]
    b_high = high[i]
    x3 = np.linspace(b_low, mid, 2000)
    x4 = np.linspace(mid, b_high , 2000)
    b_1 = 1 / (mid - b_low)
    b_2 = 1 / (b_high - mid)
    if i == 5:
        y1 = 1 * np.where((x3 < b_low) , 0, 1 / ( 1 + np.exp(-10 * (b_1*(x3 - b_low) - 0.5))))

        y2 = 1 * np.where((x4 > b_high), 0, 1 / ( 1 + np.exp(-10 * (1 - b_2*(x4 - mid) - 0.5))))
    else:
        y1 = np.zeros_like(x3)
        y2 = np.zeros_like(x4)

    plt.plot(x3, y1, label="equ1", color='blue', linewidth=7)
    plt.plot(x4, y2, label="equ1", color='blue', linewidth=7)
    #z *= -1


equ1 = "$sig1 = \\frac{1}{1 + exp(-10 \cdot (x - 0.5))}$"
ticks = np.arange(0, 1.1, 0.1)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.xticks(ticks)
ticksy = np.arange(-1, 1.25, 0.25)  # Tick-Marken von 0 bis 10 mit einem Abstand von 1
plt.yticks(ticksy)
plt.xlabel('x')
plt.ylabel('f(x)')
plt.grid()

# Display the plot
plt.show()

