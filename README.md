# ESP32-Bidirectional-DShot
A library that can do bi-directional Dshot on ESP32 and ESP32-S3 using the RMT peripheral.

Note that this code requires **two wires/ports instead of one** to do bi-directional DShot so I can set one port to TX only and one port to RX only. It seems that ESP32 will generate a glitch when switching between TX and RX, causing issues with motor controllers. 

In my application (see https://github.com/wjxway/Spinon_code), I only need to control one motor and I need absolute highest performance, so this piece of code is not well-enough packed for most cases.
However, this is the only piece of code on the internet that can do Bi-directional DShot properly at this moment, so feel free to re-organize this piece of code for your own needs. 

I will clean up the code when I have time and my drones flying...
