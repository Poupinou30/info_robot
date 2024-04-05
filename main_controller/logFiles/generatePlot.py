import matplotlib.pyplot as plt
import numpy as np

# Ouvrir le fichier en mode lecture
with open('logPosition.txt', 'r') as f:
    lines = f.readlines()

# Supprimer l'en-tête
lines = lines[1:]

# Initialiser les listes pour stocker les données
lidarPos, odometryPos, filteredPos = [], [], []

# Parcourir chaque ligne du fichier
for line in lines:
    # Extraire les données
    lidar, odometry, filtered = line.split(';')
    lidarPos.append([float(val) for val in lidar.split()])
    odometryPos.append([float(val) for val in odometry.split()])
    filteredPos.append([float(val) for val in filtered.split()])

# Convertir les listes en tableaux pour faciliter le traçage
lidarPos = np.array(lidarPos)
odometryPos = np.array(odometryPos)
filteredPos = np.array(filteredPos)

# Tracer les données
fig, axs = plt.subplots(3, 3, figsize=(15, 15))

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

# Ajouter des légendes
for i in range(3):
    for j in range(3):
        axs[i, j].legend()

# Enregistrer les tracés en JPEG
plt.savefig('plots.jpeg', format='jpeg')
