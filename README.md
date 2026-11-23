# Music Playing Doohickey
 
A compact, pocket-sized MP3 player built around an ESP32. It plays MP3s from an SD card to wired headphones or a Bluetooth speaker, and it can also act as a Bluetooth speaker itself so your phone can play through the same headphones. A small OLED screen shows what is playing, with a Portal-inspired look.
 
## Features
 
- Plays MP3s from an SD card, with next / previous / pause and shuffle
- Remembers the last song and playlist and resumes after a restart
- Three ways to listen:
  - **Wired headphones** through an I2S DAC (UDA1334A)
  - **Bluetooth speaker mode:** scan for a Bluetooth speaker or headphones and stream the SD card music to it
  - **Receiver mode:** the ESP32 becomes a Bluetooth speaker called **"Music Playing Doohickey"**. Pair your phone and its music plays through the headphones, with the song title and artist on the screen
- Playlists from `.m3u` / `.m3u8` files on the SD card
- Rotary encoder for volume, shuffle and playlists
- 128x64 OLED with animated menus and a Portal-style boot splash
## Parts
 
| Part | Notes |
|---|---|
| ESP32 development board (ESP32-CP2102) | Must be the **classic ESP32**. The C3 and S3 have no Bluetooth Classic, so A2DP will not work on them |
| UDA1334A I2S DAC with headphone jack | For the wired output |
| SD card module (SPI) | FAT32 formatted |
| 128x64 SSD1306 OLED, I2C | Power it from **3.3 V** |
| Rotary encoder with push button | KY-040 style |
| 3 push buttons | UP / DOWN / ENTER |
| 5V 2A charge / discharge module with USB-C input | Power |
| Wiring, connectors and a small enclosure | |
 
## Wiring
 
All pins match `MPD/MPD.ino`. Join all grounds together.
 
| Part | Connections (part pin -> ESP32) |
|---|---|
| UDA1334A DAC | VIN -> 5V, GND -> GND, WSEL -> GPIO 17, DIN -> GPIO 4, BCLK -> GPIO 16 |
| SD card module | CS -> GPIO 5, SCK -> GPIO 18, MISO -> GPIO 19, MOSI -> GPIO 23, VCC -> 5V, GND -> GND |
| OLED | VCC -> 3V3, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22 |
| Rotary encoder | CLK -> GPIO 32, DT -> GPIO 33, SW -> GPIO 13, GND -> GND |
| Buttons | UP -> GPIO 25, DOWN -> GPIO 26, ENTER -> GPIO 27 (other leg of each to GND) |
 
*Wiring diagram: coming soon.*
 
## Building the firmware
 
1. Install the Arduino IDE and the ESP32 board package.
2. Install the libraries: ESP32-A2DP, arduino-audio-tools, arduino-libhelix, Adafruit SSD1306, Adafruit GFX and ESP32Encoder.
3. Open `MPD/MPD.ino`, select **ESP32 Dev Module** and upload. If it says the sketch is too big, pick the **Huge APP** partition scheme.
Put your MP3s in the root folder of a FAT32 SD card. 44.1 kHz stereo works best.
 
## Using it
 
| Control | Action |
|---|---|
| UP / DOWN | Next / previous song |
| ENTER | Pause / play |
| Hold ENTER | Open the output menu |
| Turn the encoder | Volume |
| Click the encoder | Shuffle on / off |
| Hold the encoder | Open the playlist menu |
 
In menus, UP / DOWN move, ENTER selects and holding ENTER cancels.
 
**Output menu:** Earpiece (wired headphones), Bluetooth (stream to a speaker), Receiver (play from your phone) and SD card (back from receiver mode). Switching to or from receiver mode restarts the ESP32 for about 2 seconds. Your song and playlist are saved first.
 
**Receiver mode:** choose Receiver in the output menu, pair your phone with "Music Playing Doohickey", and play music on the phone. UP / DOWN / ENTER then control the phone.
 
## Playlists
 
Put `.m3u` or `.m3u8` files in the root of the SD card. The file name is the playlist name. Each line is a song:
 
```
Song Two.mp3
Song One.mp3
```
 
Lines starting with `#` are ignored. Full paths and `file:///` links from VLC and similar apps also work, because only the file name at the end is used. Songs that are not on the card are skipped. Restart the player after changing files on the card.
 
## Limitations
 
- Needs the classic ESP32 (no C3, S3 or C6)
- Only MP3 files in the root folder are played
- Bluetooth speaker mode and receiver mode cannot run at the same time

