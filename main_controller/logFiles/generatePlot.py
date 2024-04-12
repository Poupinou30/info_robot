import matplotlib.pyplot as plt
import numpy as np

# Ouvrir le fichier en mode lecture
with open('../logFiles/logPosition.txt', 'r') as f:
    lines = f.readlines()

# Supprimer l'en-tête
lines = lines[1:]

# Initialiser les listes pour stocker les données
lidarPos, odometryPos, filteredPos, opponentPos = [], [], [], []

# Parcourir chaque ligne du fichier
for line in lines:
    # Extraire les données
    data = line.split(';')
    lidar, odometry, filtered, opponent = data[0], data[1], data[2], data[3]
    lidarPos.append([float(val) for val in lidar.split()])
    odometryPos.append([float(val) for val in odometry.split()])
    filteredPos.append([float(val) for val in filtered.split()])
    opponentPos.append([float(val) for val in opponent.split()])

# Convertir les listes en tableaux pour faciliter le traçage
lidarPos = np.array(lidarPos)
odometryPos = np.array(odometryPos)
filteredPos = np.array(filteredPos)
opponentPos = np.array(opponentPos)

# Tracer les données
fig, axs = plt.subplots(4, 3, figsize=(15, 20))

# Tracer les positions x
axs[0, 0].plot(lidarPos[:, 0], label='Lidar X Position')
axs[0, 1].plot(odometryPos[:, 0], label='Odometry X Position')
axs[0, 2].plot(filteredPos[:, 0], label='Filtered X Position')

# Tracer les positions y
axs[1, 0].plot(lidarPos[:, 1], label='Lidar Y Position')
axs[1, 1].plot(odometryPos[:, 1], label='Odometry Y Position')
axs[1, 2].plot(filteredPos[:, 1], label='Filtered Y Position')

# Tracer les positions theta
axs[2, 0].plot(lidarPos[:, 2], label='Lidar Theta Position')
axs[2, 1].plot(odometryPos[:, 2], label='Odometry Theta Position')
axs[2, 2].plot(filteredPos[:, 2], label='Filtered Theta Position')

# Tracer les positions de l'opposant
axs[3, 0].plot(opponentPos[:, 0], label='Opponent X Position')
axs[3, 1].plot(opponentPos[:, 1], label='Opponent Y Position')

# Ajouter des légendes
for i in range(4):
    for j in range(3):
        axs[i, j].legend()

# Regrouper les graphiques de odometryPos, filteredPos et lidarPos
axs[0, 1].clear()
axs[1, 1].clear()
axs[2, 1].clear()

# Tracer les positions x de odometryPos, filteredPos et lidarPos
axs[0, 1].plot(lidarPos[:, 0], label='Lidar X Position', color='red')
axs[0, 1].plot(odometryPos[:, 0], label='Odometry X Position', color='green')
axs[0, 1].plot(filteredPos[:, 0], label='Filtered X Position', color='blue')

# Tracer les positions y de odometryPos, filteredPos et lidarPos
axs[1, 1].plot(lidarPos[:, 1], label='Lidar Y Position', color='red')
axs[1, 1].plot(odometryPos[:, 1], label='Odometry Y Position', color='green')
axs[1, 1].plot(filteredPos[:, 1], label='Filtered Y Position', color='blue')

# Tracer les positions theta de odometryPos, filteredPos et lidarPos
axs[2, 1].plot(lidarPos[:, 2], label='Lidar Theta Position', color='red')
axs[2, 1].plot(odometryPos[:, 2], label='Odometry Theta Position', color='green')
axs[2, 1].plot(filteredPos[:, 2], label='Filtered Theta Position', color='blue')

# Ajouter des légendes aux nouveaux graphiques
axs[0, 1].legend()
axs[1, 1].legend()
axs[2, 1].legend()

# Enregistrer les tracés en JPEG
plt.savefig('plotsMainController.jpeg', format='jpeg')
