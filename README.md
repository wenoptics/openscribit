

## Original firmware behavior

If you are starting fresh in 2025, especially after shutdown of the product - here are some good to know things.

- The left part of the strip LED light is a button. It can be used to reset the device
- The default wifi password for original firmware is `ScribItAP314`
- After connection to AP, `http://192.168.240.1:8888/` should be available.

- LED light status: [docs/support-scribit-design/led-status.md]()
- More support doc archive in [docs/support-scribit-design]()


## HowTo

### Prepare

Prepare configuration files:

```bash
# Copy example configuration files
cp ExtraFile/SIConfig.hpp.example Firmware/ScribitESP/SIConfig.hpp
cp ExtraFile/Mk4duoVersion.h.example Firmware/MK4duo/Mk4duoVersion.h  
cp ExtraFile/ScribitVersion.hpp.example Firmware/ScribitESP/ScribitVersion.hpp

# Copy required libraries
cp -r ExtraFile/arduino-mqtt Firmware/ScribitESP/
cp -r ExtraFile/StepperDriver Firmware/ScribitESP/
```

### Prepare the device:

#### New Wi-Fi Configuration
- Connect to the `ScribIt-AP` AP.
- Send a POST request to `http://192.168.240.1:8888`. The body must contain a JSON formatted as follows: `{ "ssid": "networkSSID", "password": "networkPsk" }`.
  e.g. using `curl`:
  ```bash
  curl -X POST -H "Content-Type: application/json" -d '{"ssid": "my_wifi_ssid", "password": "my_wifi_password"}' http://192.168.240.1:8888/
  ```

- The device blinks faster and responds:
  - **200**: The request is correct. The body contains a JSON formatted as follows: `{"ID":"id_device"}`.
  - **400**: Error in the request. The body contains details about the error in the format: `{"error":"error", "ID":"id_device"}`.
- If the connection is successful, the device turns the LEDs green and reboots; otherwise, it turns them red for 2 seconds and waits for a new configuration packet.

#### Delete Saved Wi-Fi
To reset the Wi-Fi configuration, press the button for at least 2 seconds. The device will reboot.


### Compile the Firmware

More detail, see [docker/README.md](docker/README.md) for more details.

- ESP32 firmware
  ```bash
  docker-compose -f docker/docker-compose.yml run --rm scribit-firmware arduino-cli compile --fqbn briki:mbc-wb:mbc:mcu=esp --output-dir /workspace/builds /workspace/source/Firmware/ScribitESP/ScribitESP.ino
  ```
  You should find `docker/builds/ScribitESP.ino.bin` after the build.
  
- SAMD21 firmware  
  ```bash
  docker-compose -f docker/docker-compose.yml run --rm scribit-firmware arduino-cli compile --fqbn briki:mbc-wb:mbc:mcu=samd --output-dir /workspace/builds /workspace/source/Firmware/MK4duo/MK4duo.ino
  ```
  You should find `docker/builds/MK4duo.ino.bin` after the build.


### Flash the Firmware with OTA

If you are connected to the Scribit AP, the robot should be accessible at `192.168.240.1` on port `3232` without a password. Or if the robot is connected to your Wi-Fi, you can find its IP address in your router's DHCP client list and access it on port `3232`.

- Upload the ESP32 firmware:
  ```bash
  python vendor/mbc-wb_2.0.0/tools/espota.py -i $ROBOT_IP_ADDRESS -p 3232 -f docker/builds/ScribitESP.ino.bin
  ```
- Upload the SAMD21 firmware:
  ```bash
  python vendor/mbc-wb_2.0.0/tools/espota.py -i $ROBOT_IP_ADDRESS -p 3232 -c -f docker/builds/MK4duo.ino.bin
  ```
- Update the ESP32 partition table:
  ```bash
  python vendor/mbc-wb_2.0.0/tools/espota.py -i $ROBOT_IP_ADDRESS -p 3232 -s -f docker/builds/ScribitESP.ino.partitions.bin
  ```

## Known Bugs
- If you perform an update from a link on SAMD with the serial monitor open, the port may become inaccessible until the first reboot.

## Troubleshooting
- The device reports insufficient space even if the GCODE is much smaller than 5MB:
  - Follow **SDK Installation** and try flashing again.
  - Follow the procedure for flashing the partition table, ensuring that this firmware for the ESP does not coexist with the one for the SAMD partition table.
- After downloading, the robot does not move, and the debug shows many serial errors:
  - Follow **SDK Installation** and verify that the `SERCOM.cpp` file has been correctly overwritten.

## Acknowledgments

- [@kris-sum](https://github.com/kris-sum)
- [scribit-open/open-firmware](https://github.com/scribit-open/open-firmware)
- [@lqu](https://github.com/lqu/scribit)
