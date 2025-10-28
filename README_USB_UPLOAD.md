ESP32-S3-WROOM-1 — USB upload instructions

This project targets the ESP32-S3-WROOM-1 on a custom PCB and uploads over USB
using the chip's native USB CDC (virtual COM port). Add or follow these steps:

1) PlatformIO config
- `platformio.ini` already sets `board = esp32-s3-wroom-1` and
  `upload_protocol = esptool` with `upload_port = auto`.
- If PlatformIO doesn't detect the port, find the COM port in Windows Device
  Manager and set `upload_port = COMn` (replace COMn with the correct number).

2) PCB wiring for auto-reset (recommended)
To allow automatic entering of the bootloader during upload, wire the USB-to-serial
bridge RTS and DTR signals to the ESP32-S3's EN (RESET) and IO0 (BOOT) pins
following Espressif's recommended circuit:

- DTR -> 100nF cap -> IO0 (also pulled-up with 10k)
- RTS -> 100nF cap -> EN (RESET)
- EN pulled-up to 3.3V via 10k
- IO0 pulled-up to 3.3V via 10k

This wiring uses the RTS/DTR toggles performed by esptool to reset the chip and
enter the serial bootloader automatically.

3) Manual upload (if you don't have RTS/DTR auto-reset)
- Hold (press and hold) the BOOT/IO0 button before starting the upload.
- Start the upload in PlatformIO.
- When you see the flashing "waiting for download" messages, release BOOT.
- If required, briefly press the EN (reset) button to reboot into bootloader.

4) Troubleshooting
- If upload fails, verify the device enumerates as a USB serial device in Device
  Manager. If it doesn't, check the USB DP/DM D+ D- connections and pull-ups.
- Try lowering `upload_speed` in `platformio.ini` to 115200.
- If the board uses an external USB-UART bridge (CP210x/FTDI/CH340), use its
  COM port in `upload_port` instead of the S3 native port.

5) Example `platformio.ini` snippet

[env:esp32-s3-wroom-1]
platform = espressif32
board = esp32-s3-wroom-1
framework = arduino
upload_protocol = esptool
upload_port = COM3    ; change to your port or use auto
upload_speed = 115200


If you want, I can also add a build/test step to verify the environment; tell me
whether you want me to run a dry build now or you'd like to try uploading first.