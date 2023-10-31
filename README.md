# Weaver_experimental_SDR

This is old code (2016) from an experimental prototype using a quadrature sampling detector frontend ("Tayloe") and a Teensy microcontroller. 
The frontend uses an Si5351 to frequency shift into the audio base band and produces the I & Q signals. The latter are digitized and fed via I2S into the Teensy, which does the mixing and demodulation.

See here for a "history" of this code:
https://forum.pjrc.com/threads/38979-Frequency-shift-with-the-Weaver-method-(-quot-third-method-quot-)-using-quadrature-oscillators?highlight=weaver

Some more resources:

https://csoundjournal.com/ezine/summer2000/processing/index.html

https://pa3ect.nl/de-derde-methode-van-opwekking-ssb-is-de-fase-weaver-methode/

https://earf.co.uk/weaver.html

http://www.hanssummers.com/weaver.html

http://www.hanssummers.com/weaver/weaverlib.html



