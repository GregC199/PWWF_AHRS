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
serial_nazwa = "COM6"
baud_rate = 115200
timeout = 100
warunek_polaczenia = 0
'''
Zmienne zwiazane z odczytem danych z portu COM
'''
numer_pliku = 0

'''
Zmienne zwiazane z odczytem danych z portu COM
'''

iterator = 0
wiersz = 0
mem = ""
kodowanie = "utf-8"
string = ""
dodatkowa_pozycja = 0
warunek = 0
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
    while True:
        rcv_data = conn.readline()
        if rcv_data:
            data_list = rcv_data.split()
            #print(data_list)
            for i in range(len(data_list)):
                string=data_list[i].decode(kodowanie)
                if string.find('.')>-1:
                    dodatkowa_pozycja = dodatkowa_pozycja + string.find('.') - 1
                    warunek = 1
                elif warunek == 0:
                    dodatkowa_pozycja = dodatkowa_pozycja + len(string)
                if len(string) < (8 + dodatkowa_pozycja):
                    print(f"string: {string}")
                    mem = mem + string
                    print(f"scalam: {mem}")
                else:
                    mem = string
                                
                
                if len(mem) > (7 + dodatkowa_pozycja):
                    print(f"powtorzenie nr {i}")
                    
                    string=mem
                    
                    iterator = iterator + 1
                    file.write(string)
                    file.write(" ")
                    mem=""
                    dodatkowa_pozycja = 0
                    warunek = 0
                    
                    if iterator > 8:
                        file.write("\n")
                        wiersz = wiersz + 1
                        iterator = 0

                
