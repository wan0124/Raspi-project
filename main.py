import thread
import time
import pigpio

MCLK_PIN=22
SCLK_PIN=23
LRCLK_PIN=24
DATA_PIN=25
f=44100
count=1
pulse=0
data=0

pi=pigpio.pi()  
if not pi.connected:
   exit(0)
    
pi.set_mode(MCLK_PIN,pigpio.OUTPUT)
pi.set_mode(SCLK_PIN,pigpio.OUTPUT)
pi.set_mode(LRCLK_PIN,pigpio.OUTPUT)
pi.set_mode(DATA_PIN,pigpio.OUTPUT)

def MCLK():
  while TRUE :
    pi.write(MCLK_PIN,pulse)
    pulse=!pulse
    count++
    time.sleep(1/(2*f))
  
def SCLK
  while TRUE :
    if pulse :
      pi.write(SCLK_PIN,1)
    else :
      pi.write(SCLK_PIN,0)

def LRCLK():
  pi.write(LRCLK_PIN,0)
  while TRUE :
    if count == 16 :
      pi.write(LRCLK_PIN,1)     
    elif count == 32:
      pi.write(LRCLK_PIN,0)
      count=1
  
def DATA():
  while TRUE :
    if pulse == 0 : 
      pi.write(DATA_PIN,data)
      data=!data
    
thread.start_new_thread (MCLK())
thread.start_new_thread (SCLK())
thread.start_new_thread (LRCLK())
thread.start_new_thread (DATA())
