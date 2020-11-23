import ADS1115
import time

ads = ADS1115.ADS1115()

while True:
    for c in range(4):
      volt = ads.readADCSingleEnded(channel=c)
      print("{:.0f} mV mesuré sur AN{}".format(volt,c))
    
    time.sleep(0.1)