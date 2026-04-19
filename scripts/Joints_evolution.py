# -*- coding: utf-8 -*-
"""
Created on Sat Aug 30 10:54:11 2025

@author: pietr
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Legge il file CSV
try:
    df = pd.read_csv('joints_data.csv', header=None)
except FileNotFoundError:
    print("Errore: il file 'joint_data.csv' non è stato trovato.")
    exit()

# Estre le colonne
# Il metodo .iloc estrae i dati in base all'indice.
# .iloc[1:] esclude la prima riga (header) se presente
positions = df.iloc[1:, 11:18].values.astype(float)
velocities = df.iloc[1:, 18:25].values.astype(float)
efforts = df.iloc[1:, 25:32].values.astype(float)

# Creazione dell'asse temporale
time = np.linspace(0, 2 * np.pi, positions.shape[0])

# Nomi dei giunti per le legende
joint_names = ['Joint 0', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']

# --------------------------------------------------
### Plot posizioni
fig_positions, axes_positions = plt.subplots(4, 2, figsize=(12, 16))

axes_positions = axes_positions.flatten()

for i in range(7):
    axes_positions[i].plot(time, positions[:, i], linewidth=1.5)
    axes_positions[i].set_title(f'Position - {joint_names[i]}')
    axes_positions[i].set_xlabel('Time [s]')
    axes_positions[i].set_ylabel('Position [rad]')
    axes_positions[i].grid(True)

#Nasconde l'ottavo subplot perché vuoto
axes_positions[7].axis('off')
plt.tight_layout()

# --------------------------------------------------
### Plot velocità

# Crea una figura con più subplot
fig_velocities, axes_velocities = plt.subplots(4, 2, figsize=(12, 16))

axes_velocities = axes_velocities.flatten()

for i in range(7):
    axes_velocities[i].plot(time, velocities[:, i], linewidth=1.5, color=(0.85, 0.33, 0.10))
    axes_velocities[i].set_title(f'Velocity - {joint_names[i]}')
    axes_velocities[i].set_xlabel('Time [s]')
    axes_velocities[i].set_ylabel('Velocity [rad/s]')
    axes_velocities[i].grid(True)
    
axes_velocities[7].axis('off')

plt.tight_layout()
plt.show()

# --------------------------------------------------
### Plot sforzi

fig_efforts, axes_efforts = plt.subplots(4, 2, figsize=(12, 16))
axes_efforts = axes_efforts.flatten()

for i in range(7):
    axes_efforts[i].plot(time, efforts[:, i], linewidth=1.5, color=(0.49, 0.18, 0.56))
    axes_efforts[i].set_title(f'Effort - {joint_names[i]}')
    axes_efforts[i].set_xlabel('Time [s]')
    axes_efforts[i].set_ylabel('Effort [Nm]')
    axes_efforts[i].grid(True)

axes_efforts[7].axis('off')

plt.tight_layout()
plt.show()