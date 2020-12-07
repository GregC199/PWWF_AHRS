'''
Created on 22 lis 2020

@author: ciesl
'''
import serial
from array import *
import os
from _ast import Try
import os, sys

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

'''
Sprawdzenie na poczatku czy nawiazanie polaczenia odbylo sie poprawnie
Jesli nie to wyswietlany jest napis informujacy o bledzie i konczacy tym samym program
'''
try:
    conn = serial.Serial(serial_nazwa,baud_rate,timeout=0)
    warunek_polaczenia = 1
except:
    print(f"Nie udalo sie nawiazac polaczenia z portem {serial_nazwa}")
    exit()
'''
Jesli udalo sie otworzyc polaczenie z portem COM to zaczynamy glowna petle programu
'''
if warunek_polaczenia == 1:
    print(f'Polaczono z {serial_nazwa}')
    '''
    Sprawdzamy wpierw czy istnieje plik o danej nazwie.
    Nazwa pliku jest generowana w nastepujacy sposob:
    W zaleznosci od wartosci zmiennej numer_pliku
    sprawdzamy czy istnieje ju¿ plik o nazwie
    "danenumer_pliku.txt"
    Jesli istnieje to zwiekszamy wartosc zmiennej i szukamy
    pierwszej wolnej nazwy.
    Jesli nazwa jest wolna to tworzymy plik i otwieramy go
    do zapisu oraz wyswietlamy napis na wyjsciu standardowym
    informujacy o powodzeniu operacji i nazwie pliku.
    '''
    while os.path.exists(f'dane{numer_pliku}.txt'):
        numer_pliku += 1
    file = open(f'dane{numer_pliku}.txt','w')
    print(f'Utworzono plik dane{numer_pliku}.txt')
    
    '''
    Nieskonczona petla programu wewnatrz, ktorej odbywa sie
    odbior informacji.
    Dane sa odczytywane z portu COM - serial_nazwa. 
    Wazna czescia algorytmu odbioru danych jest
    sprawdzanie polozenia . w kazdej z liczb, gdyz
    sa wysylane z dokladnoscia do 10^(-6). W ten sposob
    uodparniamy nasze dane na bledy w komunikacji.
    Calosc wiadomosci jest scalana przy pomocy zmiennych
    string oraz mem.
    Jest to mozliwe dzieki stalej liczbie znakow po "."
    wewnatrz kazdej z liczb.
    Na koniec zapisujemy liczby do pliku po 9 komorek 
    na linie pliku, czyli w nastepujacym formacie:
    gyr_x gyr_y gyr_z acc_x acc_y acc_z mag_x mag_y mag_z
    gdzie:
    gyr - oznacza dane z zyroskopu,
    acc - oznacza dane z akcelerometru
    mag - oznacza dane z magnetometru
    '''
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

                
