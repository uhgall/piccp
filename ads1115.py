import ADS1115
import time

ads = ADS1115.ADS1115()

while True:
    for c in range(4):
      s = 0
      for i in range(100):
        s += ads.readADCSingleEnded(channel=c)
      volt = s / 100.0

      print("{:.0f} mV mesuré sur AN{}".format(volt,c))
    
    time.sleep(0.1)