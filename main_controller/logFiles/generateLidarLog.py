import os
import matplotlib.pyplot as plt

def read_log_file(filename):
    data = {'filtered_pos_x': [], 'filtered_pos_y': [], 'filtered_pos_theta': [],
            'my_pos_x': [], 'my_pos_y': [], 'my_pos_theta': [],
            'opponent_pos_x': [], 'opponent_pos_y': [],
            'opponent_angle': [], 'opponent_distance': [],
            'distance_from_center': []}
    
    with open(filename, 'r') as file:
        lines = file.readlines()[1:-1]
        for line in lines:
            values = line.strip().split(';')
            filtered_pos = [float(val) for val in values[0].split()]
            my_pos = [float(val) for val in values[1].split()]  # Extract myPos
            opponent_pos = [float(val) for val in values[2].split()]
            opponent_angle= [float(val) for val in values[3].split()]
            opponent_distance = [float(val) for val in values[4].split()]



            distance_from_center = float(values[4])  # Extract distance_from_center

            data['filtered_pos_x'].append(filtered_pos[0])
            data['filtered_pos_y'].append(filtered_pos[1])
            data['filtered_pos_theta'].append(filtered_pos[2])
            data['my_pos_x'].append(my_pos[0])
            data['my_pos_y'].append(my_pos[1])
            data['my_pos_theta'].append(my_pos[2])
            data['opponent_pos_x'].append(opponent_pos[0])
            data['opponent_pos_y'].append(opponent_pos[1])
            data['opponent_angle'].append(opponent_angle)
            data['opponent_distance'].append(opponent_distance)
            data['distance_from_center'].append(distance_from_center)
    
    return data

def plot_data(data):
    plt.figure(figsize=(12, 12))

    plt.subplot(4, 2, 1)
    plt.plot(data['filtered_pos_x'], label='Filtered Position X')
    plt.plot(data['my_pos_x'], label='My Position X')  # Plot myPos X
    plt.legend()

    plt.subplot(4, 2, 2)
    plt.plot(data['filtered_pos_y'], label='Filtered Position Y')
    plt.plot(data['my_pos_y'], label='My Position Y')  # Plot myPos Y
    plt.legend()

    plt.subplot(4, 2, 3)
    plt.plot(data['filtered_pos_x'], data['filtered_pos_y'], label='Filtered Position XY')
    plt.plot(data['my_pos_x'], data['my_pos_y'], label='My Position XY')  # Plot myPos XY
    plt.xlabel('Filtered Position X')
    plt.ylabel('Filtered Position Y')
    plt.legend()

    plt.subplot(4, 2, 4)
    plt.plot(data['opponent_pos_x'], label='Opponent Position X')
    plt.legend()

    plt.subplot(4, 2, 5)
    plt.plot(data['opponent_pos_y'], label='Opponent Position Y')
    plt.legend()

    plt.subplot(4, 2, 6)
    plt.plot(data['opponent_pos_x'], data['opponent_pos_y'], 'ro', label='Opponent Position XY')
    plt.xlabel('Opponent Position X')
    plt.ylabel('Opponent Position Y')
    plt.legend()

    plt.subplot(4, 2, 7)
    plt.plot(data['opponent_angle'], label='Opponent Angle')
    plt.xlabel('Time')
    plt.ylabel('Angle')
    plt.legend()

    plt.subplot(4, 2, 8)
    plt.plot(data['opponent_distance'], label='Opponent Distance')
    plt.plot(data['distance_from_center'], label='Distance From Center')
    plt.xlabel('Time')
    plt.ylabel('Distance')
    plt.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(os.getcwd(), 'plots_combined.png'))
    plt.show()

if __name__ == "__main__":
    filename = "logLidar.txt"
    log_data = read_log_file(filename)
    plot_data(log_data)
