# Weaver_experimental_SDR

This is old code (2016) from an experimental prototype using a quadrature sampling detector frontend ("Tayloe") and a Teensy microcontroller. 
The frontend uses an Si5351 to frequency shift into the audio base band and produces the I & Q signals. The latter are digitized and fed via I2S into the Teensy, which does the mixing and demodulation.

