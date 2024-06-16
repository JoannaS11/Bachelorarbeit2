import matplotlib.pyplot as plt
import numpy as np


def plot_vectors(vector_to_line, pcd_colon, start_points):
    fig = plt.figure()
    print("hallo")

    ax = fig.add_subplot(111, projection='3d')
    width = 0.01
    ax.quiver(pcd_colon[:,0], pcd_colon[:,1], pcd_colon[:,2], vector_to_line[:,0], vector_to_line[:,1], vector_to_line[:,2])
    ax.plot(start_points[:,0], start_points[:,1], start_points[:,2], color = 'r')
    # Set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    print("here")
    # Rotate the axes and update
    for angle in range(0, 1):#90):
        # Normalize the angle to the range [-180, 180] for display
        angle_norm = (angle + 180) % 360 - 180

        # Cycle through a full rotation of elevation, then azimuth, roll, and all
        elev = azim = roll = 0
        if angle <= 360:
            elev = angle_norm
        elif angle <= 360*2:
            azim = angle_norm
        elif angle <= 360*3:
            roll = angle_norm
        else:
            elev = azim = roll = angle_norm

        # Update the axis view and title
        ax.view_init(0, 0, 0)
        plt.title('Elevation: %d°, Azimuth: %d°, Roll: %d°' % (elev, azim, roll))

        plt.draw()
        #plt.pause(.001)


    plt.show()


def main():
    x = np.linspace(-4 * np.pi, 4 * np.pi, 50)
    y = np.linspace(-4 * np.pi, 4 * np.pi, 50)
    z = x ** 2 + y ** 2
    vector_to_line = np.array([[1,2,3],[3,5,4], [7,8,9]])
    pcd_colon = np.zeros([3,3])
    plot_vectors(vector_to_line, pcd_colon, np.array([x,y,z]))
    """ plt.rcParams["figure.figsize"] = [7.50, 3.50]
    plt.rcParams["figure.autolayout"] = True
    x = np.linspace(-4 * np.pi, 4 * np.pi, 50)
    y = np.linspace(-4 * np.pi, 4 * np.pi, 50)
    z = x ** 2 + y ** 2
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #ax.plot(x, y, z)
    ax.quiver(0,0,0, x,y,z)
    plt.show()


    fig = plt.figure()
    #ax = plt.axes(projection = '3d')
    vector_to_line = np.array([[1,2,3],[3,5,4], [7,8,9]])
    pcd_colon = np.zeros([3,3])
    print(vector_to_line)
    ax = fig.add_subplot(111, projection='3d')

    ax.quiver(pcd_colon[:,0], pcd_colon[:,1], pcd_colon[:,2], vector_to_line[:,0], vector_to_line[:,1], vector_to_line[:,2])
    # Set the axis labels
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # Rotate the axes and update
    for angle in range(0, 360*4 + 1):
        # Normalize the angle to the range [-180, 180] for display
        angle_norm = (angle + 180) % 360 - 180

        # Cycle through a full rotation of elevation, then azimuth, roll, and all
        elev = azim = roll = 0
        if angle <= 360:
            elev = angle_norm
        elif angle <= 360*2:
            azim = angle_norm
        elif angle <= 360*3:
            roll = angle_norm
        else:
            elev = azim = roll = angle_norm

        # Update the axis view and title
        ax.view_init(elev, azim, roll)
        plt.title('Elevation: %d°, Azimuth: %d°, Roll: %d°' % (elev, azim, roll))

        plt.draw()
        plt.pause(.001)

    plt.show()


    sleep(10)"""

if __name__ == "__main__": main()