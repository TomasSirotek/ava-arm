# AVA bill of materials

Electronics and servos of the minimal hobby build. Prices from dratek.cz (October 2026).

## To buy

| Item | Product (dratek.cz) | Qty | Price CZK |
|---|---|---|---|
| Servos J1–J3 (shoulder pan, shoulder lift, elbow) | [Servo MG996R, metal gears](https://dratek.cz/arduino-platforma/1694-servo-motor-s-kovovymi-prevody-mg996r-pro-rc-modely.html) | 3 | 3 × 137 = 411 |
| Servo driver | [PCA9685, 16 channels](https://dratek.cz/arduino-platforma/1686-iic-i2c-modulovy-driver-servo-motoru-pro-arduino-pca9685-16-kanalu-12-bit-pwm.html) | 1 | 167 |
| Step-down converter (24 V → 6.0 V servo rail) | [XL4016E1, 8 A](https://dratek.cz/arduino-platforma/180340-step-down-menic-4-40-v-na-1.5-36-v-xl4016e1-max.-8-a-200-w.html) | 1 | 64 |
| ON/OFF switch (24 V side) | [Toggle switch with lock, 6 A](https://dratek.cz/arduino-platforma/1080-packovy-prepinac-6a-125vac.html) | 1 | 34 |
| DC socket | [DC 5.5/2.1 female](https://dratek.cz/arduino-platforma/124099-napajeci-kabelovy-konektor-dc-5.5-2.1-samice.html) (12 CZK, 5 A) or [DC 5.5/2.5 female](https://dratek.cz/arduino-platforma/124098-napajeci-kabelovy-konektor-dc-5.5-2.5-samice.html) (7 CZK): match the adapter's plug; skip it if you cut the plug off | 1 | 12 |
| **Total** | | | **688 CZK / 28.23 EUR** |

Optional: one 1000 µF / 16 V capacitor across V+ and GND at the PCA9685 terminal if the servos
twitch when several start together (the board already has one fitted).

## Already owned

| Item | Used for | Qty used |
|---|---|---|
| 24 V DC 2 A adapter | power for the servo rail (through the step-down converter) | 1 |
| ESP32-C3-DevKitM-1-N4X | USB serial ↔ PCA9685 (I2C on GPIO0/GPIO1), powered by USB | 1 |
| SG90 9g servos (8 in stock) | J4 wrist flex, J5 wrist roll, J6 gripper | 3 |
| Low-voltage cables, DuPont wires | wiring | — |

## Not listed yet

Printed parts, bearings, screws and spacers: they follow the Fusion 360 design, which is still
changing (elbow MG996R mount being redrawn). Add them once the CAD export is final.
