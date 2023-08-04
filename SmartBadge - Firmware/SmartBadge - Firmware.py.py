# LSM6DSOX Gyro example.
import time , sensor, image, machine , os ,sdcard ,pyb , audio ,tf , math, uos, gc , mpyaes, ucryptolib
from lsm6dsox import LSM6DSOX
from pyb import LED , ADC , USB_VCP, RTC
from machine import I2C, SPI, Pin
from vl53l1x import VL53L1X
from ulab import numpy as np
import math
from rsa.randnum import read_random_bits


#Mise à jour de la RTC
def RTC():
    #Initialisation RTC et USB
    etape=0             #Variable definissant les differentes étapes de communications
    RTCvalue = rtc.datetime()
    if(usb.isconnected()==True):    #Verification de la connexion entre le niclavision et le pc
            if(etape==0):           #Etape 0 attente de la reception de "un" permettant le début de la com
                                    #tant que on recoit pas le "un" aucune connexion ne sera effectuée
                cmd = usb.recv(2, timeout=5000)
                if (cmd == b'un'):
                    green_led.on()
                    etape=1

            if(etape==1):                   #Envoie du RTC pour comparaison sur le PC
                 usb.send(str(RTCvalue))
                 etape=2
                 print("etape2")

            if(etape==2):                           #Etape 2 Reception du RTC et division des différentes part
                 cmd = usb.recv(4, timeout=5000)
                 if (cmd == b'deux'):
                    cmd = usb.recv(26, timeout=5000)    #Reception du RTC
                    usb.send(cmd)
                    UsbRTC = str(cmd)
                    annee = int(UsbRTC[2:6])            #Division par partie du RTC
                    mois =  int(UsbRTC[7:9])
                    jour =  int(UsbRTC[10:12])
                    heure = int(UsbRTC[13:15])
                    minute =int(UsbRTC[16:18])
                    seconde =int(UsbRTC[19:21])
                    rtc.datetime((annee, mois, jour,0, heure, minute, seconde, 0))
                    etape=3

            if(etape==3):           #Etape 3 envoie du RTC pour verification de la value
                time.sleep_ms(200)
                coucou = rtc.datetime()
                usb.send(str(RTCvalue))
                time.sleep_ms(1000)
                green_led.off()
                blue_led.on()
                etape=0

#Conversion d'un format bit à un format long utilisé pour le cryptage de la clé AES
def bytes_to_long(s):
    n = s[0]
    for b in (x for x in s[1:]):
        n = (n << 8) | b
    return n

#Fonction permettant l'enrengistrement du son, le son est enrengistré sous format pcm
def audio_callback(buf):
    try:
        global fout
        pcm_buf = np.frombuffer(buf, dtype=np.int16)
        fout.write(pcm_buf*200)
    except Exception as e:
        print("exption:", e)

#Fonction appelé lors d'une interruption en appuyant sur le bouton
#Elle permet de demarrer ou d'arreter le systeme,et de verifier l'etat actuel du system
def handle_interrupt(pin):
  global ButtonCurrentValue
  if(ButtonCurrentValue==1):
        select = time.ticks_ms()
        while(pir.value()==1):
            if(time.ticks_ms()- select>3000):   #Si on appuie plus de 3s sur le bouton le systeme s'arrete
                ButtonCurrentValue=2
                green_led.on()                  #On allume la lumiere blanche pour verifier que le sys est stop
                red_led.on()
                blue_led.on()
                time.sleep_ms(3000)
                green_led.off()                 #Une fois que la LED blanche est éteinte alors le systeme est
                red_led.off()                   #Remis à son état d'origine
                blue_led.off()
  else:
    ButtonCurrentValue = 1

#Initialisation de la caméra et des LED
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Image en gris
sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others)
sensor.set_windowing((320, 320))  # Set 240x240 window.
sensor.skip_frames(time = 2000)   # Let new settings take affect.
clock = time.clock()
green_led = LED(2)                #Intialisation des 3 LEDS
red_led = LED(1)
blue_led = LED(3)
BaseTxt = "Horodatage   AccX   AccY   AccZ  GyrX    GyrY    GyrZ    \n" #Txt d'en tête du fichier txt

#Initialisations du module sonor
VarMicro = 0
AllumageMicro = 0
audio.init(channels=1, frequency=16000, gain_db=120, highpass=0.9883)
#Information : Changé la fréquence, le gain et le hight pass ne modifie rien

#Initialisation de l'ADC et de l'I2C
i2c = I2C(1)
lsm = LSM6DSOX(SPI(5), cs_pin=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP), gyro_odr=104, accel_odr=104, gyro_scale=500, accel_scale=4)
tof = VL53L1X(I2C(2))

#Initialisation de la carte SD
try:    #On verifie que une carte SD est presente si oui on l'initialise
    sd = sdcard.SDCard(pyb.SPI(4), pyb.Pin.board.PE11)
    vfs = os.VfsFat(sd)
    os.mount(vfs, "/fc")
    print(os.listdir("/fc"))
    BufferStockageIMU = ""
    BufferIMU =""
    variableBufferImu=0
    fn = "/fc/AccGyro2.txt"
    select = time.ticks_ms()
    VariableSDCARD=1
except OSError: #Dans le cas ou la carte SD n'est pas présente alors le code ne fonctionneras pas
    VariableSDCARD=0

#Initialisation de l'ALS
AddrALS = 0x44  #Adresse de l'ALS quand relié à la masse
TimeBuf = bytearray(2)
Buffer = bytearray([0xB4,0x00])
octet_pF = 0
octet_pf=0
i2c.writeto_mem(AddrALS,0x01,Buffer)

#Initialisation du modele crypto
KeyBase = b'(D\xde\x12uT\x8a\xa9\x84\xa7\xc4\x89d\x0f\xc4\xbf4\xe9i\x1e/I\x80\xe6\x8b\x0e\x165C\x06\xa4f'
Id = machine.unique_id()   #Recuperaiton de l'ID du micro
Key = KeyBase+Id
IV =  b'1\xb9U9\xe7+{%s \xd9\xaf\xb8\xc0\x17\xb3'
aes = mpyaes.new(KeyBase, mpyaes.MODE_CBC, IV)      #Creation de la clé AES

if(VariableSDCARD==1):                       #Si une carte SD est présente alors on ecrit sinon on ne fait rien
    with open('/fc/cryptageKey.txt','r') as f:  #On recupere la clé publique RSA
        n = f.read()
        f.close()
    e=65537
    m= bytes_to_long(KeyBase)
    c=pow(m,e, int(n))

    with open('/fc/AesKey.txt','w') as f:   #On ecrit la clé AES crypter par la clé RSA dans la carte SD
        f.write(str(c))
        f.close()
    with open(fn, "a") as f:                #Ecriture du haut de page sur le fichier txt
              n = f.write(BaseTxt)

#Initialisation Interruption et RTC
pir = Pin("PC4", Pin.IN)
pir.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt)   #On initialise le bouton pour appeler la fonction à
ButtonCurrentValue=0                                        #chaque appuie
rtc = pyb.RTC()
usb = USB_VCP()

#Initialisation varible
i=0
b=0
VarCamera=1
VarDodo = 0
etape=0
exists=True
red_led.on()
while(True):

    red_led.on()
    #Verification de la présence de la connexion entre le PC et le NiclaVision
    if(usb.isconnected()==True):
            RTC()

#Si la carte SD est présente
    if(VariableSDCARD==1):
#Quand le bouton est appuyé lancement du code
     if(ButtonCurrentValue == 1):
        ButtonCurrentValue = 0
        time.sleep_ms(500)
        green_led.off()
        red_led.off()
        blue_led.off()

#Tant que le bouton n'est pas réapuyé le code tournera
        while(ButtonCurrentValue!=2):
            clock.tick()
            i=i+1
            #Toutes les X secondes nous allons prendre une photo qui sera save dans la carte SD ainsi que une
            #prise de mesure de l'ALS et du TOF:
            if(VarMicro == 0):  #Variable de securité permettant de verifier que le micro n'est pas utilisé
                if(VarCamera == 1):
                        time.sleep_ms(300)
                        img = sensor.snapshot() #Prise d'une photo
                        time.sleep_ms(300)
                        VarCamera=0
                        b=b+1
                        while(exists!=False):   #Verification de l'existence du numéro d'une photo
                            try:
                                f = open("/fc/Image" + str(b)+".jpg", "r")
                                exists = True
                                print("/fc/Image" + str(b)+".jpg")
                                print("OUI")
                                b=b+1           #On incrémente b jusqu'a arriver à la valeur de l'image
                                f.close()
                            except OSError:
                                exists = False

                        img.save("/fc/example"+str(b)+".jpg")   #On sauvegarde l'image
                        VarMicro = 1        #On informe le code que on peut demarrer le micro

                        #Récupération de données de l'ALS
                        i2c.readfrom_mem_into(AddrALS, 0x00, TimeBuf)
                        octet_pF = TimeBuf[0] >> 4
                        octet_pf = (0x0F)&(TimeBuf[0])
                        Lumiere = (octet_pf<<8)|(TimeBuf[1])
                        Lux = 0.01 * math.pow(2,octet_pF)*Lumiere
                        Lumino = str(int(Lux))
                        ToF = f"{tof.read()    }"

            #Nous allumons le micro pendant X secondes et nous récupérons les valeurs des différents capteurs
            if(VarMicro == 1):

                 #Prise de mesure de l'accelero et du gyro
                 Accelero = str(b)+","+str(i)+ ' {:>8.3f}    {:>8.3f}    {:>8.3f}    '.format(*lsm.read_accel())
                 Gyroscope = '{:>8.3f}    {:>8.3f}    {:>8.3f}    '.format(*lsm.read_gyro())
                 RTCvaleur = rtc.datetime()
                 Total = Accelero + Gyroscope + "\n"
                 BufferStockageIMU = BufferStockageIMU + Total
                 #print(len(BufferStockageIMU))
                  #Remplissage du Buffer avec les données des capteurs
                 if(len(BufferStockageIMU)>=11000):
                        BufferIMU = BufferStockageIMU
                        BufferStockageIMU =""
                        variableBufferImu=1

                 TitreImage = "/fc/Image" + str(b)+".jpg"


                 #Enrengistrement audio
                 if(AllumageMicro ==0): #Variable permettant de lancer une fois l'enrengistrement audio
                    print("RECORD")
                    TitreSon ="/fc/Audio" + str(b) + "_" + str(RTCvaleur)+ "_" + ToF + "_" + Lumino + '.pdm'
                    fout = open(TitreSon, 'w')
                    audio.start_streaming(audio_callback)
                    select = time.ticks_ms()
                    AllumageMicro = 1

                 #Apres X temps le micro s'eteint en un fichier audio et save dans la carte SD
                 if((time.ticks_ms() - select) > 2000):
                       VarMicro = 0
                       VarDodo = 1
                       audio.stop_streaming()
                       AllumageMicro = 0
                       fout.close()
                       aes.encrypt_file("/fc/example"+str(b)+".jpg",TitreImage)
                       os.remove("/fc/example"+str(b)+".jpg")

            #Quand l'enrengistrement audio est fini et que le buffer est remplit,
            #enrengisrement sur la carte SD des valeurs des différents capteurs
            if(((VarMicro == 0))and (variableBufferImu ==1)):
                with open(fn, "a") as f:
                    f.write(BufferIMU)
                variableBufferImu = 0
                BufferIMU=""


            #Mode sleep une fois l'enrengistrement audio terminé
            if(VarDodo == 1):
                green_led.off()
                blue_led.on()
                print("zzzzzz")
                machine.sleep(7500)
                #time.sleep_ms(7500)
                print("reveil")
                VarCamera=1
                blue_led.off()
                green_led.on()
                VarDodo = 0
            time.sleep_ms(10)

        #Quand on quitte la boucle de lecture la led change de couleur et le programme attend que l'utilisateur
        #réappuie sur le bouton pour lancer un nouvel enrengistrement
        green_led.off()
        red_led.off()
        blue_led.off()
        audio.stop_streaming()
        fout.close()

        #Enrengistrement de la fin du buffer dans la carteSD
        with open(fn, "a") as f:
                    n = f.write(BufferStockageIMU)
                    BufferStockageIMU = ""

        print("FINITO")



