# LSM6DSOX Gyro example.
import time , sensor, image, machine , os ,sdcard ,pyb , audio ,tf , math, uos, gc , mpyaes, ucryptolib
from lsm6dsox import LSM6DSOX
from pyb import LED , ADC
from machine import I2C, SPI, Pin
from vl53l1x import VL53L1X
from ulab import numpy as np
import math
from rsa.randnum import read_random_bits

def bytes_to_long(s):
    n = s[0]
    for b in (x for x in s[1:]):
        n = (n << 8) | b
    return n

def audio_callback(buf):    #Fonction permettant l'enrengistrement du son
    try:
        global fout
        pcm_buf = np.frombuffer(buf, dtype=np.int16)
        fout.write(pcm_buf*200)
    except Exception as e:
        print("exption:", e)

#Initialisation de la caméra et des LED

sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Image en gris
sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others)
sensor.set_windowing((320, 320))       # Set 240x240 window.
#sensor.set_windowing((100, 100))
sensor.skip_frames(time = 2000) # Let new settings take affect.
clock = time.clock()
green_led = LED(2)
red_led = LED(1)


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
#--i2c.writeto_mem(AddrALS,0x01,Buffer)

#Initialisation du modele crypto
KeyBase = b'(D\xde\x12uT\x8a\xa9\x84\xa7\xc4\x89d\x0f\xc4\xbf4\xe9i\x1e/I\x80\xe6\x8b\x0e\x165C\x06\xa4f'
Id = machine.unique_id()
Key = KeyBase+Id
IV =  b'1\xb9U9\xe7+{%s \xd9\xaf\xb8\xc0\x17\xb3'
aes = mpyaes.new(KeyBase, mpyaes.MODE_CBC, IV)

with open('valeurn.txt','r') as f:
    n = f.read()
    f.close()

print(n)
e=65537

m= bytes_to_long(KeyBase)
print(m)
c=pow(m,e, int(n))

with open('AesKey.txt','w') as f:
    f.write(str(c))
    f.close()

#Initialisation varible
i=0
b=0

red_led.on()

while(True):
     ButtonCurrentValue = int(((adc.read() * 3.3) / 4095))
     time.sleep_ms(100)

     if(ButtonCurrentValue == 3):
        red_led.off()
        green_led.on()
        time.sleep_ms(4000)
        ButtonLastValue = 0
        ButtonCurrentValue = 0
        start = time.ticks_ms()
        print(ButtonCurrentValue)
        while(ButtonCurrentValue<3):
            clock.tick()
            i=i+1
            #Toutes les secondes nous allons prendre une photo :
            if(VarMicro == 0):
                img = sensor.snapshot()
                if((time.ticks_ms() - start) > 5000):
                        b=b+1
                        red_led.on()
                        img.save("/fc/example"+str(b)+".jpg")
                        aes.encrypt_file("/fc/example"+str(b)+".jpg","/fc/Crypte"+str(b)+".jpg")
                        os.remove("/fc/example"+str(b)+".jpg")
                        print("Petit sourire pour la photo")
                        VarMicro = 1
                        start = time.ticks_ms()

            #Directement apres la prise d'une photo nous allumons le micro
            if(VarMicro == 1):

                 if(AllumageMicro ==0):
                    print("Nouveau fichier ")
                    print("RECORD")
                    fout = open('/fc/Audio' + str(b) + '.pdm', 'w')
                    print("Initialisaiton fichier audio")
                    audio.start_streaming(audio_callback)
                    select = time.ticks_ms()
                    AllumageMicro = 1


                 if((time.ticks_ms() - select) > 2000): #Apres x temps le micro est eteint
                       VarMicro = 0
                       print("FIN DU RECORD")
                       audio.stop_streaming()
                       AllumageMicro = 0
                       red_led.off()
                       fout.close()



            #Prise de mesure de l'ALS
            """
            i2c.readfrom_mem_into(AddrALS, 0x00, TimeBuf)
            octet_pF = TimeBuf[0] >> 4
            octet_pf = (0x0F)&(TimeBuf[0])
            Lumiere = (octet_pf<<8)|(TimeBuf[1])
            Lux = 0.01 * math.pow(2,octet_pF)*Lumiere
            Lumino = "  lum : " + str(Lux) + " Lux \n"
            """
            Lumino =""
            #Prise de mesure de l'accelero et du gyro

            Accelero = str(b)+","+str(i)+ '\n Acc: x:{:>8.3f} y:{:>8.3f} z:{:>8.3f} \n '.format(*lsm.read_accel())
            Gyroscope = ' Gyr: x:{:>8.3f} y:{:>8.3f} z:{:>8.3f}\n '.format(*lsm.read_gyro())
            ToF = f" Dist: {tof.read()}mm \n"

            Total = Accelero + Gyroscope + ToF + Lumino

            if(((VarMicro == 0))and (variableHero ==1)):
                with open(fn, "a") as f:
                    f.write(Hero)
                variableHero = 0

            if(len(FINAL)>=10000):
                print("SALUT")
                Hero = FINAL
                FINAL =""
                variableHero=1

            FINAL = FINAL + Total
            time.sleep_ms(10)

            ButtonCurrentValue = int(((adc.read() * 3.3) / 4095))


        print(ButtonCurrentValue)
        print(ButtonLastValue)
        green_led.off()
        red_led.off()
        audio.stop_streaming()
        fout.close()

        with open(fn, "a") as f:
                    n = f.write(FINAL)
                    FINAL = ""
        print("FINITO")

     time.sleep_ms(1000)
     ButtonCurrentValue=0
