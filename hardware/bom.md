# AVA bill of materials

Electronics and servos of the Ava arm.

| Item | Part | Qty | Key specs | Price CZK | Price EUR |
|---|---|---|---|---|---|
| Servos J1–J3 (shoulder pan, shoulder lift, elbow) | [Servo MG996R, metal gears](https://dratek.cz/arduino-platforma/1694-servo-motor-s-kovovymi-prevody-mg996r-pro-rc-modely.html) | 3 | 4.8–7.2 V; torque 9.4 kg·cm @ 4.8 V, 11 kg·cm @ 6 V; 0.17 s/60° @ 4.8 V, 0.13 s/60° @ 6 V; 40 × 19 × 43 mm, 55 g | 3 × 137 = 411 | 16.90 |
| Servos J4–J6 (wrist flex, wrist roll, gripper) | SG90 9g micro servo | 3 | torque 1.2 kg·cm @ 4.8 V, 1.6 kg·cm @ 6 V | — | — |
| Servo driver | [Servo shield PCA9685 PWM, 16 channels](https://rpishop.cz/669047/servo-shield-pca9685-pwm-16-kanalu) | 1 | I2C, 12-bit PWM, 40–1000 Hz; external servo supply up to 6 V; 60 × 25 mm | 192 | 7.89 |
| Controller | ESP32-C3-DevKitM-1-N4X | 1 | USB-powered; I2C to the PCA9685 | — | — |
| Power supply | 24 V DC adapter | 1 | 24 V, 2 A (48 W) | — | — |
| Step-down converter (6.0 V servo rail) | [XL4016E1 step-down](https://dratek.cz/arduino-platforma/180340-step-down-menic-4-40-v-na-1.5-36-v-xl4016e1-max.-8-a-200-w.html) | 1 | input 4–40 V DC; output adjustable 1.5–36 V; max 8 A / 200 W | 64 | 2.63 |
| Power switch (24 V side) | [Rocker switch RS-201-1C3-R/B](https://dratek.cz/arduino-platforma/177621-kolebkovy-spinac-rs-201-1c3-r-b.html) | 1 | 2-pole ON/OFF, 15 A / 250 V; 6.3 mm faston tabs; mounting hole 21.7 × 28.5 mm | 19 | 0.78 |
| DC socket | [DC connector 5.5 × 2.1 mm female, spring terminal](https://dratek.cz/arduino-platforma/175936-dc-konektor-s-pruzinovou-svorkou-5-5-x-2-1-mm-samice.html) | 1 | 5.5 × 2.1 mm, + / − marked | 15 | 0.62 |
| **Total** | | | | **701** | **28.82** |

Optional, not counted: one 1000 µF / 16 V capacitor across V+ and GND at the PCA9685 terminal if
the servos twitch when several start together (the board already has one fitted).
