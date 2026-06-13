# Numbat Electronics Setup

After reading multiple repos, many different ways to wire out SpotMicro. Micheals is the most complete and in my opinion the safest however his wiring is designed for the ESP32 with a number of added safety features to protect the ESP32 and connected sensors from current and voltage spikes. Most repos using a Raspberry Pi to implement SpotMirco assume you are an electronics God and don't need much guidence with a number of safety features being removed. The end result for me was confusion and delay.

Reading all the repos I found that every implementation uses the PCA9685 servo driver, a 5V Voltage regulator to power the raspberry Pi/ ESP32 and some sort of Buck Converter/BEC to power the PCA9685.

Below is our current circuit diagram that powers our setup.

## Documentation
- PCA9685 [Module Datasheet](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf)
- PCA9685 [Driver Datasheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
- MG996R [Servo Datasheet](https://components101.com/sites/default/files/component_datasheet/MG996R%20Datasheet.pdf)
- DS3218 [20kg Servo Datasheet](https://images-na.ssl-images-amazon.com/images/I/81Lbgu+nG6L.pdf)
- DS3230 [30kg Servo Datasheet](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.rc-dome.de/file/pdf/fileId/5724&ved=2ahUKEwjs2qTE1YOVAxUNaPUHHebTOJYQFnoECB4QAQ&usg=AOvVaw36H4uSm27pboyZXBGlpDFG)

## Bill of Materials (TBA)

## Current Management
I am concerned the servos will draw too much current through the PCA9685 board that it will break or overheat the board. The PCA9685 datasheet makes it quite clear the maximum voltage passing through V+ for the servo's be no more than 6V with mutliple internet sources stating the board will break at any voltage above [7.2V](https://www.reddit.com/r/arduino/comments/zi6a4u/providing_more_power_to_pca9685_16_channel_servo/). The 20A SBEC exists to protect the board from voltage and current overload.

Inspection of the PCA9685 [datatsheet](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf) shows the PCA9685 driver and servo rail are powered seperately the driver requires a 3.3V supply while the servo rail takes upto 6V.

However, the datasheet does not state the maximum permissible current through the servo rail, the data sheet reconmends using a 5V 10A power module to power the board which suggests that the servo power rail at the minium can take 10 Amps [datasheet page 10](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf). The documentation also shows the PCB schematic which shows a large rail seperated isolated from the rest of the board suggesting large currents can be taken by the board. 
<img width="1844" height="1096" alt="image" src="https://github.com/user-attachments/assets/ebc11d4f-7e59-41b0-9aaa-b9aa1eb37a1c" />

Given the stall current of each MG996R servo is 2.1A the group resolved to connect the servo directly to the PCA9685 Servo rail if issues arise we will will inact one of the contingencies below. I note: most documentation state the MG996R servos are insufficent for operation for the spotmicro robtoic with even the orginal ESP32 repo suggesting to use 20kg servo's. If we upgrade to 20kg servos then the stall current would increase to 2.5A and may also require inplementing one of the following contingencies.

**Contingencies include**:
- Powering each servo directly through a custom power distribution board - Internal idea never seen done.
- Powering the servos rail directly as done (here)[] using the two SBEC current sources
- Using two PCA9685 boards can be connected in series. Last resort documentation allows for this [Page 28](https://cdn-learn.adafruit.com/downloads/pdf/16-channel-pwm-servo-driver.pdf).
