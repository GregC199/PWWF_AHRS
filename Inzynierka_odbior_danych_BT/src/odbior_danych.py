'''
Created on 22 lis 2020

@author: ciesl
'''
import serial
from array import *
import os
from _ast import Try

'''
Zmienne zwiazane portem COM
'''
serial_nazwa = 'COM6'
baud_rate = 115200
timeout = 100
warunek_polaczenia = 0
'''
Zmienne zwiazane z odczytem danych z portu COM
'''
numer_pliku = 0
'''


                              Poczatek programu glownego


'''
try:
    conn = serial.Serial(serial_nazwa,baud_rate,timeout=0)
    warunek_polaczenia = 1
except:
    print(f"Nie udalo sie nawiazac polaczenia z portem {serial_nazwa}")

if warunek_polaczenia == 1:
    print(f'Polaczono z {serial_nazwa}')
    while os.path.exists(f'dane{numer_pliku}.txt'):
        numer_pliku += 1
    file = open(f'dane{numer_pliku}.txt','w')
    print(f'Utworzono plik dane{numer_pliku}.txt')
