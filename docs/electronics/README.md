# Numbat Electronics Setup

After reading multiple repos, many different ways to wire out SpotMicro. Micheals is the most complete and in my opinion the safest however his wiring is designed for the ESP32 with a number of added safety features to protect the ESP32 and connected sensors from current and voltage spikes. Most repos using a Raspberry Pi to implement SpotMirco assume you are an electronics God and don't need much guidence with a number of safety features being removed. The end result for me was confusion and delay.

Reading all the repos I found that every implementation uses the PCA9685 servo driver, a 5V Voltage regulator to power the raspberry Pi or ESP32 and some sort of Buck Converter/BEC to power the PCA9685. 
