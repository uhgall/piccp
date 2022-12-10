# piccp

Controller for cathodic corrosion protection / mineral accretion experiments in the ocean. Basically a multi channel programmable current source for negative voltages (ie, with a common positive instead of a common GND). 

Runs on a raspberry pi with a modified buck converter where the voltage adjustment potentiometer was replaced with the resistor side of an optocoupler. The idea was to have multiple channels for the cathode, with a shared anode, so they all needed a common positive. 

Got it to work, application level code is currently just for one channel, then got distracted. 
