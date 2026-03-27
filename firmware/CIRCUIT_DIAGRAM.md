# V2V Communication System — Circuit Diagram

## Components
| # | Component | Description |
|---|-----------|-------------|
| 1 | **ESP32 DevKit v1** | Dual-core MCU — central controller |
| 2 | **NRF24L01+** | 2.4 GHz RF transceiver (SPI) |
| 3 | **NEO-6M GPS** | UART GPS module (9600 baud) |
| 4 | **SSD1306 OLED** | 128×64 I2C display (0.96") |
| 5 | **Active Buzzer** | 3.3V active buzzer + NPN transistor driver |
| 6 | **LEDs** | Red, Green, Blue — 3mm with 220Ω resistors |

## Wiring Diagram

```
                         ┌──────────────────────┐
                         │     ESP32 DevKit v1   │
                         │                       │
    NRF24L01+            │                       │          NEO-6M GPS
   ┌─────────┐           │                       │         ┌──────────┐
   │ VCC ────┼──── 3.3V ─┤ 3V3             3V3 ├──── VCC ─┤ VCC      │
   │ GND ────┼──── GND ──┤ GND             GND ├──── GND ─┤ GND      │
   │ CE  ────┼───────────┤ GPIO 4               │          │          │
   │ CSN ────┼───────────┤ GPIO 5          GPIO16├─────────┤ TX       │
   │ SCK ────┼───────────┤ GPIO 18         GPIO17├─────────┤ RX       │
   │ MOSI────┼───────────┤ GPIO 23              │          └──────────┘
   │ MISO────┼───────────┤ GPIO 19              │
   │ IRQ     │ (unused)  │                       │
   └─────────┘           │                       │
                         │                       │
    SSD1306 OLED         │                       │          Alert LEDs
   ┌──────────┐          │                       │
   │ VCC ─────┼── 3.3V ──┤ 3V3                  │    ┌──[220Ω]──LED(RED)──GND
   │ GND ─────┼── GND ───┤ GND            GPIO26├────┘
   │ SDA ─────┼──────────┤ GPIO 21               │    ┌──[220Ω]──LED(GRN)──GND
   │ SCL ─────┼──────────┤ GPIO 22         GPIO27├────┘
   └──────────┘          │                       │    ┌──[220Ω]──LED(BLU)──GND
                         │                 GPIO14├────┘
                         │                       │
                         │                       │          Active Buzzer
                         │                       │
                         │                 GPIO25├────[1kΩ]──B┐
                         │                       │            │ (NPN 2N2222)
                         │                  3.3V ├──[Buzzer+]─C┘
                         │                   GND ├──[Buzzer-]─E──GND
                         │                       │
                         └───────────────────────┘
```

## Pin Summary

| ESP32 Pin | Connected To | Protocol |
|-----------|-------------|----------|
| GPIO 4 | NRF24L01 CE | SPI (control) |
| GPIO 5 | NRF24L01 CSN | SPI (control) |
| GPIO 18 | NRF24L01 SCK | SPI (clock) |
| GPIO 23 | NRF24L01 MOSI | SPI (data out) |
| GPIO 19 | NRF24L01 MISO | SPI (data in) |
| GPIO 16 | NEO-6M TX | UART2 RX |
| GPIO 17 | NEO-6M RX | UART2 TX |
| GPIO 21 | SSD1306 SDA | I2C (data) |
| GPIO 22 | SSD1306 SCL | I2C (clock) |
| GPIO 25 | Buzzer (via transistor) | Digital out |
| GPIO 26 | Red LED (via 220Ω) | Digital out |
| GPIO 27 | Green LED (via 220Ω) | Digital out |
| GPIO 14 | Blue LED (via 220Ω) | Digital out |

## Power Notes

> ⚠ **NRF24L01+ power**: Add a **10µF electrolytic capacitor** across VCC and GND of the NRF24L01+ module, as close to the module as possible. This prevents voltage droop during TX bursts.

> ⚠ **OLED power**: The SSD1306 runs at 3.3V. Most breakout boards have onboard regulators and can also accept 5V — check your module.

> ⚠ **Buzzer driver**: Use an NPN transistor (2N2222 or BC547) as a switch. GPIO25 drives the base through a 1kΩ resistor. The buzzer connects between 3.3V and the collector. If you're using a small 3.3V buzzer that draws < 12mA, you can drive it directly from GPIO25 without a transistor.

## I2C Pull-ups

Most SSD1306 OLED breakout boards include built-in 4.7kΩ pull-up resistors on SDA and SCL. If your module doesn't, add external **4.7kΩ pull-ups to 3.3V** on both lines.
