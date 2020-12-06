import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
from pyquaternion import Quaternion

file = open("dane0.txt","r")

dt = 0.02

liczba_wierszy = 0

gyr_x = []
gyr_y = []
gyr_z = []

acc_x = []
acc_y = []
acc_z = []

mag_x = []
mag_y = []
mag_z = []

czas = []

komplementarny_x = []
komplementarny_y = []
komplementarny_z = []

kalman_x = []
kalman_y = []
kalman_z = []

mahony_x = []
mahony_y = []
mahony_z = []

madgwick_x = []
madgwick_y = []
madgwick_z = []
    
for linia in file:
    dane = linia.split()
    
    gyr_x.append(float(dane[0]))
    gyr_y.append(float(dane[1]))
    gyr_z.append(float(dane[2]))
    
    acc_x.append(float(dane[3]))
    acc_y.append(float(dane[4]))
    acc_z.append(float(dane[5]))
    
    mag_x.append(float(dane[6]))
    mag_y.append(float(dane[7]))
    mag_z.append(float(dane[8]))
    
    czas.append(dt*liczba_wierszy)
    
    liczba_wierszy += 1

def oblicz_roll_pitch_yaw(q):
    q._normalise()
    roll = np.arctan2(2 * (q[0] * q[1] - q[2] * q[3]),1 - 2 * (q[1] ** 2 + q[2] ** 2))
    pitch = np.arcsin(2 * (q[0] * q[2] + q[3] * q[1]))
    yaw = np.arctan2(2 * (q[0] * q[3] - q[1] * q[2]),1 - 2 * (q[2] ** 2 + q[3] ** 2))
    
    katy = [roll,pitch,yaw]
    
    return katy

def filtr_komplementarny(tau):
    alfa = tau/(tau+dt)
    
    roll = []
    pitch = []
    yaw = []
    
    roll.append(gyr_x[0] * dt) 
    pitch.append(gyr_y[0] * dt)
    yaw.append(gyr_z[0] * dt)
    
    normalise_mag=math.sqrt((mag_x[0]**2)+(mag_y[0]**2)+(mag_z[0]**2))
    
    rollAcc = np.arctan2(acc_x[0], acc_z[0]) * 180 / np.pi
    roll[0] = roll[0] * alfa + rollAcc * (1-alfa)

    pitchAcc = np.arctan2(acc_y[0], acc_z[0]) * 180 / np.pi
    pitch[0] = pitch[0] * alfa + pitchAcc * (1-alfa)
    
    yawMag = (np.arctan2( (-mag_y[0]*np.cos(roll[0])/normalise_mag + mag_z[0]*np.sin(roll[0])/normalise_mag  ) , (mag_x[0]*np.cos(pitch[0])/normalise_mag  + (mag_y[0]/normalise_mag)*np.sin(pitch[0])*np.sin(roll[0])+ mag_z[0]*np.sin(pitch[0])*np.cos(roll[0])/normalise_mag ) )) * 180 / np.pi
    yaw[0] = yaw[0]*alfa + yawMag*(1-alfa)
    
    for iterator in range(1,len(czas)):
        
        roll.append((roll[iterator-1]+gyr_x[iterator] * dt)) 
        pitch.append((pitch[iterator-1]+gyr_y[iterator] * dt))
        yaw.append(yaw[iterator-1]+gyr_z[iterator]*dt)
        
        normalise_mag=math.sqrt((mag_x[iterator]**2)+(mag_y[iterator]**2)+(mag_z[iterator]**2))
        
        przyrost = (acc_x[iterator]-acc_x[iterator-1]+acc_y[iterator]-acc_y[iterator-1]+acc_z[iterator]-acc_z[iterator-1])

        if przyrost > 0.4:
            rollAcc = np.arctan2(acc_x[iterator], acc_z[iterator]) * 180 / np.pi
            roll[iterator] = roll[iterator] * alfa + rollAcc * (1-alfa)
    
            pitchAcc = np.arctan2(acc_y[iterator], acc_z[iterator]) * 180 / np.pi
            pitch[iterator] = pitch[iterator] * alfa + pitchAcc * (1-alfa)
            
        yawMag=np.arctan2( (-mag_y[iterator]*np.cos(roll[iterator])/normalise_mag  + mag_z[iterator]*np.sin(roll[iterator])/normalise_mag  ) , (mag_x[iterator]*np.cos(pitch[iterator])/normalise_mag  + mag_y[iterator]*np.sin(pitch[iterator])*np.sin(roll[iterator])/normalise_mag + mag_z[iterator]*np.sin(pitch[iterator])*np.cos(roll[iterator])/normalise_mag ) )* 180 / np.pi 
        yaw[iterator] = yaw[iterator]*alfa + yawMag*(alfa-1) 
    
    return [roll,pitch,yaw]
'''
b = filtr_komplementarny()
c = oblicz_roll_pitch_yaw(b)
print(b)
print(b.get_axis())
print(b.get_axis(undefined=[0,0,0]))
print(c)
b._normalise()
print(b)'''
    
[komplementarny_x,komplementarny_y,komplementarny_z] = filtr_komplementarny(0.5)

surowe_dane=pd.DataFrame({'czas':czas,'x': gyr_x, 'y': gyr_y, 'z': gyr_z})
'''xlen = len(komplementarny_x)
ylen = len(komplementarny_y)
zlen = len(komplementarny_z)
czaslen = len(czas)
print(f'xlen: {xlen} ylen: {ylen} zlen: {zlen} czaslen: {czaslen}')
print(komplementarny_x)
print(czas)'''
komplementarny_dane=pd.DataFrame({'czas':czas,'x': komplementarny_x, 'y': komplementarny_y, 'z': komplementarny_z})

plt.plot( surowe_dane.czas, surowe_dane.x, marker='', color='skyblue', linewidth=2,label="x")
plt.plot( komplementarny_dane.czas, komplementarny_dane.x, marker='', color='green', linewidth=2,label="x_komp")
plt.axis([0,czas[liczba_wierszy-1],-360,+360])
plt.title('OX')
plt.legend()
plt.show()

plt.plot( surowe_dane.czas, surowe_dane.y, marker='', color='skyblue', linewidth=2,label="y")
plt.plot( komplementarny_dane.czas, komplementarny_dane.y, marker='', color='green', linewidth=2,label="y_komp")
plt.axis([0,czas[liczba_wierszy-1],-360,+360])
plt.title('OY')
plt.legend()
plt.show()

plt.plot( surowe_dane.czas, surowe_dane.z, marker='', color='skyblue', linewidth=2,label="z")
plt.plot( komplementarny_dane.czas, komplementarny_dane.z, marker='', color='green', linewidth=2,label="z_komp")
plt.axis([0,czas[liczba_wierszy-1],-360,+360])
plt.title('OZ')
plt.legend()
plt.show()
'''
print(f'gyr_x: {gyr_x}')
print(f'gyr_y: {gyr_y}')
print(f'gyr_z: {gyr_z}')
print(f'acc_x: {acc_x}')
print(f'acc_y: {acc_y}')
print(f'acc_z: {acc_z}')
print(f'mag_x: {mag_x}')
print(f'mag_y: {mag_y}')
print(f'mag_z: {mag_z}')


plt.plot( surowe_dane.czas, surowe_dane.x, marker='', color='skyblue', linewidth=2,label="x")
plt.plot( surowe_dane.czas, surowe_dane.y, marker='', color='green', linewidth=2,label="y")
plt.plot( surowe_dane.czas, surowe_dane.z, marker='', color='red', linewidth=2,label="z")
plt.axis([0,czas[liczba_wierszy-1],-360,+360])
plt.title('Surowe dane z zyroskopu')
plt.legend()
plt.show()'''


