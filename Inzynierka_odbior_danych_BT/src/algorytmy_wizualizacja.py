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
    roll = np.arctan2(2 * (q[0] * q[1] - q[2] * q[3]),1 - 2 * (q[1] ** 2 + q[2] ** 2))*180/np.pi
    pitch = np.arcsin(2 * (q[0] * q[2] + q[3] * q[1]))*180/np.pi
    yaw = np.arctan2(2 * (q[0] * q[3] - q[1] * q[2]),1 - 2 * (q[2] ** 2 + q[3] ** 2))*180/np.pi
    
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

def filtr_kalmana():
    
    roll = []
    pitch = []
    yaw = []
    
    return [roll,pitch,yaw]

def filtr_Mahonyego(ki,kp):
    
    roll = []
    pitch = []
    yaw = []
    
    q = Quaternion(1.0,0.0,0.0,0.0)
    
    E_m_q = Quaternion(0.0, 0.198283,-0.021548, 0.444230)
    E_a_q = Quaternion(0.0, 0.0, 0.0, 1.0)
        
    s_m_qest = q.inverse * E_m_q * q
    s_a_qest = q.inverse * E_a_q * q
    
    s_w_mes = [(acc_x[0]*s_a_qest[1] + mag_x[0]*s_m_qest[1]), (acc_y[0]*s_a_qest[2] + mag_y[0]*s_m_qest[2]), (acc_z[0]*s_a_qest[3] + mag_z[0]*s_m_qest[3])]
    
    s_w_qr1 = gyr_x[0]*dt + ki*s_w_mes[0]*dt + kp*s_w_mes[0]
    s_w_qr2 = gyr_y[0]*dt + ki*s_w_mes[1]*dt + kp*s_w_mes[1]
    s_w_qr3 = gyr_z[0]*dt + ki*s_w_mes[2]*dt + kp*s_w_mes[2]
    
    s_w_qr = Quaternion((0), (s_w_qr1), (s_w_qr2), (s_w_qr3))
    
    q12 = Quaternion(q.elements/2)
    
    q_dot = q12*s_w_qr
    
    q_k = Quaternion(q.elements + q_dot.elements*dt) 
    
    [r,p,y] = oblicz_roll_pitch_yaw(q_k)
    roll.append(r)
    pitch.append(p)
    yaw.append(y)
    
    for iterator in range(1,len(czas)):
        q = q_k
        
        s_m_q = Quaternion((0.0),(mag_x[iterator]),(mag_y[iterator]),(mag_z[iterator]))
        s_a_q = Quaternion((0.0),(acc_x[iterator]),(acc_y[iterator]),(acc_z[iterator]))
        
        s_m_qest = q.inverse * E_m_q * q
        s_a_qest = q.inverse * E_a_q * q
        
        s_w_mes1 = (acc_x[iterator]*s_a_qest[1] + mag_x[iterator]*s_m_qest[1])
        s_w_mes2 = (acc_y[iterator]*s_a_qest[2] + mag_y[iterator]*s_m_qest[2])
        s_w_mes3 = (acc_z[iterator]*s_a_qest[3] + mag_z[iterator]*s_m_qest[3])
        
        s_w_mes = [s_w_mes1, s_w_mes2, s_w_mes3]
    
        s_w_qr1 = gyr_x[iterator]*dt + ki*s_w_mes[0]*dt + kp*s_w_mes[0]
        s_w_qr2 = gyr_y[iterator]*dt + ki*s_w_mes[1]*dt + kp*s_w_mes[1]
        s_w_qr3 = gyr_z[iterator]*dt + ki*s_w_mes[2]*dt + kp*s_w_mes[2]
        
        s_w_qr = Quaternion((0), (s_w_qr1), (s_w_qr2), (s_w_qr3))
        
        q12 = Quaternion(q.elements/2)
        
        q_dot = q12*s_w_qr
        
        q_k = Quaternion(q.elements + q_dot.elements*dt)
        
        [r,p,y] = oblicz_roll_pitch_yaw(q_k)
        roll.append(r)
        pitch.append(p)
        yaw.append(y)
    
    return [roll,pitch,yaw]
    
def filtr_Madgwicka(beta,zeta):
    
    roll = []
    pitch = []
    yaw = []
    
    beta = beta * math.sqrt(3/4) * np.pi / 180
    zeta = zeta * math.sqrt(3/4) * np.pi / 180
    
    q = Quaternion(1.0,0.0,0.0,0.0)
    
    E_a_q = Quaternion(0.0, 0.0, 0.0, 1.0)
    
    s_m_q = Quaternion(0.0, (mag_x[0]), (mag_y[0]), (mag_z[0]))
    s_a_q = Quaternion(0.0, (acc_x[0]), (acc_y[0]), (acc_z[0]))
    
    E_h_q = q * s_m_q * q.inverse
    
    E_m_q = Quaternion(0.0, (math.sqrt(E_h_q[1]**2+E_h_q[2]**2)), 0.0, E_h_q[3])
    
    F_k1 = Quaternion(q.inverse * E_a_q * q - s_a_q)
    F_k2 = Quaternion(q.inverse * E_m_q * q - s_m_q)
    
    F_k = np.array([[F_k1[1]],[F_k1[2]],[F_k1[3]],[F_k2[1]],[F_k2[2]],[F_k2[3]]])
    
    J_Fk1 = [(-2*q[2]), (2*q[3]), (-2*q[0]), (2*q[1])]
    J_Fk2 = [(2*q[1]), (2*q[0]), (2*q[3]), (2*q[2])]
    J_Fk3 = [(0), (-4*q[1]), (-4*q[2]), (0)]
    J_Fk4 = [(-2*E_h_q[3]*q[2]), (2*E_h_q[3]*q[3]), (-4*E_h_q[1]*q[2]-2*E_h_q[3]*q[0]), (-4*E_h_q[1]*q[3]+2*E_h_q[3]*q[1])]
    J_Fk5 = [(-2*E_h_q[1]*q[3]+2*E_h_q[3]*q[1]), (2*E_h_q[1]*q[2]+2*E_h_q[3]*q[0]), (2*E_h_q[1]*q[1]+2*E_h_q[3]*q[3]), (-2*E_h_q[1]*q[0]+2*E_h_q[3]*q[2])]
    J_Fk6 = [(2*E_h_q[1]*q[2]), (2*E_h_q[1]*q[3]-4*E_h_q[3]*q[1]), (2*E_h_q[1]*q[0]-4*E_h_q[3]*q[2]), (2*E_h_q[1]*q[1])]
    
    JT_Fk = np.array([J_Fk1, J_Fk2, J_Fk3, J_Fk4, J_Fk5, J_Fk6]).transpose()
    
    Grad_Fk = Quaternion(JT_Fk.dot(F_k))
    
    Grad_Fk.unit
    
    s_w_eq = 2*q.inverse*Grad_Fk
    
    s_w_bq = zeta * s_w_eq * dt
    
    s_w_eeq = Quaternion((0.0), (gyr_x[0]), (gyr_y[0]), (gyr_z[0])) - s_w_bq
    
    q_dotk = q * s_w_eeq/2 - beta * Grad_Fk
    
    q_k = q + q_dotk*dt
    
    [r,p,y] = oblicz_roll_pitch_yaw(q_k)
    roll.append(r)
    pitch.append(p)
    yaw.append(y)
    
    for iterator in range(1,len(czas)):
        
        q = q_k
        
        s_m_q = Quaternion(0.0, (mag_x[iterator]), (mag_y[iterator]), (mag_z[iterator]))
        s_a_q = Quaternion(0.0, (acc_x[iterator]), (acc_y[iterator]), (acc_z[iterator]))
        
        E_h_q = q * s_m_q * q.inverse
        
        E_m_q = Quaternion(0.0, (math.sqrt(E_h_q[1]**2+E_h_q[2]**2)), 0.0, E_h_q[3])
        
        F_k1 = Quaternion(q.inverse * E_a_q * q - s_a_q)
        F_k2 = Quaternion(q.inverse * E_m_q * q - s_m_q)
        
        F_k = np.array([[F_k1[1]],[F_k1[2]],[F_k1[3]],[F_k2[1]],[F_k2[2]],[F_k2[3]]])
        
        J_Fk1 = [(-2*q[2]), (2*q[3]), (-2*q[0]), (2*q[1])]
        J_Fk2 = [(2*q[1]), (2*q[0]), (2*q[3]), (2*q[2])]
        J_Fk3 = [(0), (-4*q[1]), (-4*q[2]), (0)]
        J_Fk4 = [(-2*E_h_q[3]*q[2]), (2*E_h_q[3]*q[3]), (-4*E_h_q[1]*q[2]-2*E_h_q[3]*q[0]), (-4*E_h_q[1]*q[3]+2*E_h_q[3]*q[1])]
        J_Fk5 = [(-2*E_h_q[1]*q[3]+2*E_h_q[3]*q[1]), (2*E_h_q[1]*q[2]+2*E_h_q[3]*q[0]), (2*E_h_q[1]*q[1]+2*E_h_q[3]*q[3]), (-2*E_h_q[1]*q[0]+2*E_h_q[3]*q[2])]
        J_Fk6 = [(2*E_h_q[1]*q[2]), (2*E_h_q[1]*q[3]-4*E_h_q[3]*q[1]), (2*E_h_q[1]*q[0]-4*E_h_q[3]*q[2]), (2*E_h_q[1]*q[1])]
        
        JT_Fk = np.array([J_Fk1, J_Fk2, J_Fk3, J_Fk4, J_Fk5, J_Fk6]).transpose()
        
        Grad_Fk = Quaternion(JT_Fk.dot(F_k))
        
        Grad_Fk.unit
        
        s_w_eq = 2*q.inverse*Grad_Fk
        
        s_w_bq += zeta * s_w_eq * dt
        
        s_w_eeq = Quaternion((0.0), (gyr_x[iterator]), (gyr_y[iterator]), (gyr_z[iterator]))*dt - s_w_bq
        
        q_dotk = Quaternion(q * s_w_eeq/2 - beta * Grad_Fk)
        
        q_k = Quaternion(q + q_dotk*dt)
        
        [r,p,y] = oblicz_roll_pitch_yaw(q_k)
        roll.append(r)
        pitch.append(p)
        yaw.append(y)
    
    return [roll,pitch,yaw]

[komplementarny_x,komplementarny_y,komplementarny_z] = filtr_komplementarny(0.5)
[kalman_x,kalman_y,kalman_z] = filtr_kalmana()
[mahony_x, mahony_y, mahony_z] = filtr_Mahonyego(2,0.2)
[madgwick_x, madgwick_y, madgwick_z] = filtr_Madgwicka(5,0.2)

print(f'xlen: {madgwick_x} ylen: {madgwick_y} zlen: {madgwick_z}')

surowe_dane=pd.DataFrame({'czas':czas,'x': gyr_x, 'y': gyr_y, 'z': gyr_z})

komplementarny_dane=pd.DataFrame({'czas':czas,'x': komplementarny_x, 'y': komplementarny_y, 'z': komplementarny_z})
kalman_dane=pd.DataFrame({'czas':czas,'x': kalman_x, 'y': kalman_y, 'z': kalman_z})
mahony_dane=pd.DataFrame({'czas':czas,'x': mahony_x, 'y': mahony_y, 'z': mahony_z})
madgwick_dane=pd.DataFrame({'czas':czas,'x': madgwick_x, 'y': madgwick_y, 'z': madgwick_z})

plt.plot( surowe_dane.czas, surowe_dane.x, marker='', color='skyblue', linewidth=2,label="x")
plt.plot( komplementarny_dane.czas, komplementarny_dane.x, marker='', color='green', linewidth=2,label="x_komp")
plt.plot( mahony_dane.czas, mahony_dane.x, marker='', color='blue', linewidth=2,label="x_mah")
plt.plot( madgwick_dane.czas, madgwick_dane.x, marker='', color='red', linewidth=2,label="x_mad")
#plt.plot( kalman_dane.czas, kalman_dane.x, marker='', color='orange', linewidth=2,label="x_kal")
plt.axis([0,czas[liczba_wierszy-1],-360,+360])
plt.title('OX')
plt.legend()
plt.show()

plt.plot( surowe_dane.czas, surowe_dane.y, marker='', color='skyblue', linewidth=2,label="y")
plt.plot( komplementarny_dane.czas, komplementarny_dane.y, marker='', color='green', linewidth=2,label="y_komp")
plt.plot( mahony_dane.czas, mahony_dane.y, marker='', color='blue', linewidth=2,label="y_mah")
plt.plot( madgwick_dane.czas, madgwick_dane.y, marker='', color='red', linewidth=2,label="y_mad")
#plt.plot( kalman_dane.czas, kalman_dane.y, marker='', color='orange', linewidth=2,label="y_kal")
plt.axis([0,czas[liczba_wierszy-1],-360,+360])
plt.title('OY')
plt.legend()
plt.show()

plt.plot( surowe_dane.czas, surowe_dane.z, marker='', color='skyblue', linewidth=2,label="z")
plt.plot( komplementarny_dane.czas, komplementarny_dane.z, marker='', color='green', linewidth=2,label="z_komp")
plt.plot( mahony_dane.czas, mahony_dane.z, marker='', color='blue', linewidth=2,label="z_mah")
plt.plot( madgwick_dane.czas, madgwick_dane.z, marker='', color='red', linewidth=2,label="z_mad")
#plt.plot( kalman_dane.czas, kalman_dane.z, marker='', color='orange', linewidth=2,label="z_kal")
plt.axis([0,czas[liczba_wierszy-1],-360,+360])
plt.title('OZ')
plt.legend()
plt.show()