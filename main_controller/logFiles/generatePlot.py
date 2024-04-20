import matplotlib.pyplot as plt
import numpy as np

# Ouvrir le fichier en mode lecture
with open('../logFiles/logPosition.txt', 'r') as f:
    lines = f.readlines()

# Supprimer l'en-tête
lines = lines[1:]

# Initialiser les listes pour stocker les données
lidarPos, odometryPos, filteredPos, opponentPos, opponentFilteredPos, measuredOmega, filteredOmega = [], [], [], [], [], [], []

# Parcourir chaque ligne du fichier
for line in lines:
    # Extraire les données
    data = line.split(';')
    lidar, odometry, filtered, opponent, opponent_filtered, measuredOmegaLines, filteredOmegaLines = data[0], data[1], data[2], data[3], data[4], data[5], data[6]
    lidarPos.append([float(val) for val in lidar.split()])
    odometryPos.append([float(val) for val in odometry.split()])
    filteredPos.append([float(val) for val in filtered.split()])
    opponentPos.append([float(val) for val in opponent.split()])
    opponentFilteredPos.append([float(val) for val in opponent_filtered.split()])
    measuredOmega.append([float(val) for val in measuredOmegaLines.split()])
    filteredOmega.append([float(val) for val in filteredOmegaLines.split()])

# Convertir les listes en tableaux pour faciliter le traçage
lidarPos = np.array(lidarPos)
odometryPos = np.array(odometryPos)
filteredPos = np.array(filteredPos)
opponentPos = np.array(opponentPos)
opponentFilteredPos = np.array(opponentFilteredPos)
measuredOmega = np.array(measuredOmega)
filteredOmega = np.array(filteredOmega)

# Tracer les données
fig, axs = plt.subplots(7, 3, figsize=(15, 35))  # Mise à jour pour 7 rangées de graphiques

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

# Tracer les positions de l'opposant en x
axs[3, 0].plot(opponentPos[:, 0], label='Opponent X Position')

# Tracer les positions de l'opposant en y
axs[4, 0].plot(opponentPos[:, 1], label='Opponent Y Position')

# Tracer les positions filtrées de l'opposant
axs[5, 0].plot(opponentFilteredPos[:, 0], label='Opponent Filtered X Position')
axs[5, 1].plot(opponentFilteredPos[:, 1], label='Opponent Filtered Y Position')

# Tracer la vitesse angulaire mesurée
axs[6, 0].plot(measuredOmega, label='Measured Omega')

# Tracer la vitesse angulaire filtrée
axs[6, 1].plot(filteredOmega, label='Filtered Omega')

# Ajouter des légendes à tous les graphiques
for ax_row in axs:
    for ax in ax_row:
        ax.legend()

# Enregistrer les tracés en JPEG
plt.savefig('plotsMainController.jpeg', format='jpeg')
plt.show()
