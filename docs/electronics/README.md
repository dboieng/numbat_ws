# Numbat Electronics Setup

After reading multiple repos, many different ways to wire out SpotMicro. Micheals is the most complete and in my opinion the safest however his wiring is designed for the ESP32 with a number of added safety features to protect the ESP32 and connected sensors from current and voltage spikes. Most repos using a Raspberry Pi to implement SpotMirco assume you are an electronics God and don't need much guidence with a number of safety features being removed. The end result for me was confusion and delay.

Reading all the repos I found that every implementation uses the PCA9685 servo driver, a 5V Voltage regulator to power the raspberry Pi/ ESP32 and some sort of Buck Converter/BEC to power the PCA9685.

Below is our current circuit diagram that powers our setup.

## Documentation
- PCA9685 [Module Datasheet](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf)
- PCA9685 [Driver DataSheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
- MG996R [Servo Datasheet](https://components101.com/sites/default/files/component_datasheet/MG996R%20Datasheet.pdf)
- DS3218 [20kg Servo Datasheet](https://images-na.ssl-images-amazon.com/images/I/81Lbgu+nG6L.pdf)
- DS3230 [30kg Servo Datasheet](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.rc-dome.de/file/pdf/fileId/5724&ved=2ahUKEwjs2qTE1YOVAxUNaPUHHebTOJYQFnoECB4QAQ&usg=AOvVaw36H4uSm27pboyZXBGlpDFG)


## Current Management
I am concerned the servos will draw too much current through the PCA9685 board that it will break or overheat the board. The PCA9685 datasheet makes it quite clear the maximum voltage passing through V+ for the servo's be no more than 6V with mutliple internet sources stating the board will break at any voltage above 7.2V. The 20A SBEC exists to protect the board from voltage and current overload.

Inspection of the PCA9685 [datatsheet](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf) shows the PCA9685 driver and servo rail are powered seperately the driver requires a 3.3V supply while the servo rail takes upto 6V.

However, the datasheet does not state the maximum permissible current through the servo rail, the data sheet reconmends using a 5V 10A power module to power the board which suggests that the servo power rail at the minium can take 10 Amps (datasheet page 10).

**Contingencis include**:
- Powering each servo directly through a custom power distribution board.
- Powering the servos rail directly as done (here)[] using the two SBEC current sources
- Using two PCA9685 boards can be connected in series. (Last resort documentation allows for this)
