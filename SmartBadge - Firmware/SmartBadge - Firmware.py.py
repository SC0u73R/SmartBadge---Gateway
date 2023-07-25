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
    #rtc = RTC()
    #usb = USB_VCP()
    etape=0
    coucou = rtc.datetime()
    if(usb.isconnected()==True):
            #time.sleep_ms(1000)
            if(etape==0):

                #salut = usb.send(b"Salut")
                blue_led.on()
                cmd = usb.recv(2, timeout=5000)
                if (cmd == b'un'):
                    green_led.on()
                    etape=1
                    #usb.send(b"Miaou")
            if(etape==1):
                 usb.send(str(coucou))
                 etape=2
                 print("etape2")
            if(etape==2):
                 cmd = usb.recv(4, timeout=5000)
                 if (cmd == b'deux'):
                    cmd = usb.recv(26, timeout=5000)
                    usb.send(cmd)
                    bonjour = str(cmd)
                    annee = int(bonjour[2:6])
                    mois =  int(bonjour[7:9])
                    jour =  int(bonjour[10:12])
                    heure = int(bonjour[13:15])
                    minute =int(bonjour[16:18])
                    seconde =int(bonjour[19:21])
                    rtc.datetime((annee, mois, jour,0, heure, minute, seconde, 0))
                    etape=3

            if(etape==3):
                time.sleep_ms(1000)
                green_led.off()
                etape=0

#Conversion d'un format bit à un format long
def bytes_to_long(s):
    n = s[0]
    for b in (x for x in s[1:]):
        n = (n << 8) | b
    return n

#Fonction permettant l'enrengistrement du son
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
  print("SALUT")
  global ButtonCurrentValue
  if(ButtonCurrentValue==1):
        select = time.ticks_ms()
        while(pir.value()==1):
            if(time.ticks_ms()-select>2000):
                ButtonCurrentValue=2
            if(ButtonCurrentValue!=2):
                green_led.on()
                red_led.on()
                time.sleep_ms(1000)
                green_led.off()
                red_led.off()

  else:
    ButtonCurrentValue = 1

#Initialisation de la caméra et des LED
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Image en gris
sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others)
sensor.set_windowing((320, 320))       # Set 240x240 window.
sensor.skip_frames(time = 2000) # Let new settings take affect.
clock = time.clock()
green_led = LED(2)
red_led = LED(1)
blue_led = LED(3)
BaseTxt = "Horodatage   AccX   AccY   AccZ  GyrX    GyrY    GyrZ    \n"

#Initialisations du module sonor
VarMicro = 0
AllumageMicro = 0
audio.init(channels=1, frequency=16000, gain_db=120, highpass=0.9883)

#Initialisation de l'ADC et de l'I2C
adc = ADC("A0")
i2c = I2C(1)
lsm = LSM6DSOX(SPI(5), cs_pin=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP), gyro_odr=104, accel_odr=104, gyro_scale=500, accel_scale=4)
tof = VL53L1X(I2C(2))

#Initialisation de la carte SD
sd = sdcard.SDCard(pyb.SPI(4), pyb.Pin.board.PE11)
vfs = os.VfsFat(sd)
os.mount(vfs, "/fc")
print(os.listdir("/fc"))
FINAL = ""
Hero = ""
variableHero=0
fn = "/fc/AccGyro2.txt"
select = time.ticks_ms()

#Initialisation de l'ALS
AddrALS = 0x44
TimeBuf = bytearray(2)
Buffer = bytearray([0xB4,0x00])
octet_pF = 0
octet_pf=0
i2c.writeto_mem(AddrALS,0x01,Buffer)

#Initialisation du modele crypto
KeyBase = b'(D\xde\x12uT\x8a\xa9\x84\xa7\xc4\x89d\x0f\xc4\xbf4\xe9i\x1e/I\x80\xe6\x8b\x0e\x165C\x06\xa4f'
Id = machine.unique_id()
Key = KeyBase+Id
IV =  b'1\xb9U9\xe7+{%s \xd9\xaf\xb8\xc0\x17\xb3'
aes = mpyaes.new(KeyBase, mpyaes.MODE_CBC, IV)

with open('/fc/cryptageKey.txt','r') as f:
    n = f.read()
    f.close()
e=65537
m= bytes_to_long(KeyBase)
c=pow(m,e, int(n))

with open('/fc/AesKey.txt','w') as f:
    f.write(str(c))
    f.close()

#Initialisation Interruption et RTC
pir = Pin("PC4", Pin.IN)
pir.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt)
ButtonCurrentValue=0
rtc = pyb.RTC()
usb = USB_VCP()

#Initialisation varible
i=0
b=0
bonjour=0
VarCamera=1
VarDodo = 0
etape=0
Horo=""
exists=True
red_led.on()
with open(fn, "a") as f:
              n = f.write(BaseTxt)

while(True):
     #ButtonCurrentValue = int(((adc.read()* 3.3) / 4095))
     red_led.on()
     if(usb.isconnected()==True):
            #blue_led.on()
            RTC()

#Quand le bouton est appuyé lancement du code
     if(ButtonCurrentValue == 1):
        ButtonLastValue = 0
        ButtonCurrentValue = 0
        time.sleep_ms(2000)
        start = time.ticks_ms()

#Tant que le bouton n'est pas réapuyé le code tournera
        while(ButtonCurrentValue!=2):
            #green_led.on()
            #red_led.off()
            clock.tick()
            i=i+1

            #Toutes les X secondes nous allons prendre une photo qui sera save dans la carte SD:
            if(VarMicro == 0):
                if(VarCamera == 1):
                        time.sleep_ms(300)
                        img = sensor.snapshot()
                        time.sleep_ms(300)
                        print(time.ticks_ms() - bonjour)
                        VarCamera=0
                        b=b+1
                        while(exists!=False):
                            try:
                                f = open("/fc/Image" + str(b)+".jpg", "r")
                                exists = True
                                print("/fc/Image" + str(b)+".jpg")
                                print("OUI")
                                b=b+1
                                f.close()
                            except OSError:
                                exists = False

                        img.save("/fc/example"+str(b)+".jpg")
                        VarMicro = 1
                        start = time.ticks_ms()

            #Nous allumons le micro pendant X secondes et nous récupérons les valeurs des différents capteurs
            if(VarMicro == 1):

                 #Récupération de données de l'ALS
                 i2c.readfrom_mem_into(AddrALS, 0x00, TimeBuf)
                 octet_pF = TimeBuf[0] >> 4
                 octet_pf = (0x0F)&(TimeBuf[0])
                 Lumiere = (octet_pf<<8)|(TimeBuf[1])
                 Lux = 0.01 * math.pow(2,octet_pF)*Lumiere
                 Lumino = str(int(Lux))

                 #Prise de mesure de l'accelero et du gyro
                 Accelero = str(b)+","+str(i)+ ' {:>8.3f}    {:>8.3f}    {:>8.3f}    '.format(*lsm.read_accel())
                 Gyroscope = '{:>8.3f}    {:>8.3f}    {:>8.3f}    '.format(*lsm.read_gyro())
                 ToF = f"{tof.read()    }"


                 coucou = rtc.datetime()
                 Total = Accelero + Gyroscope + "\n"
                 FINAL = FINAL + Total
                 TitreImage = "/fc/Image" + str(b)+".jpg"


                 #Enrengistrement audio
                 if(AllumageMicro ==0):
                    print("RECORD")
                    TitreSon ="/fc/Audio" + str(b) + "_" + str(coucou)+ "_" + ToF + "_" + Lumino + '.pdm'
                    fout = open(TitreSon, 'w')
                    #fout = open('/fc/Audio' + str(b) + '.pdm', 'w')
                    audio.start_streaming(audio_callback)
                    select = time.ticks_ms()
                    AllumageMicro = 1

                 #Apres X temps le micro s'eteint en un fichier audio et save dans la carte SD
                 if((time.ticks_ms() - select) > 2000):

                       VarMicro = 0
                       VarDodo = 1
                       bonjour = time.ticks_ms()
                       audio.stop_streaming()
                       AllumageMicro = 0
                       fout.close()
                       aes.encrypt_file("/fc/example"+str(b)+".jpg",TitreImage)
                       os.remove("/fc/example"+str(b)+".jpg")

            #Quand l'enrengistrement audio est fini et que le buffer est remplit,
            #enrengisrement sur la carte SD des valeurs des différents capteurs
            if(((VarMicro == 0))and (variableHero ==1)):
                with open(fn, "a") as f:
                    f.write(Hero)
                variableHero = 0

            #Remplissage du Buffer avec les données des capteurs
            if(len(FINAL)>=10000):
                Hero = FINAL
                FINAL =""
                variableHero=1

            #Mode sleep une fois l'enrengistrement audio terminé
            if(VarDodo == 1):

                #blue_led.on()
                print("zzzzzz")
                machine.sleep(7500)
                print("reveil")
                #red_led.on()
                VarCamera=1
                #blue_led.off()
                VarDodo = 0
            time.sleep_ms(10)

        #Quand on quitte la boucle de lecture la led change de couleur et le programme attend que l'utilisateur
        #réappuie sur le bouton pour lancer un nouvel enrengistrement
        green_led.off()
        red_led.off()
        blue_led.off()
        audio.stop_streaming()
        fout.close()

        with open(fn, "a") as f:
                    n = f.write(FINAL)
                    FINAL = ""
        print("FINITO")



