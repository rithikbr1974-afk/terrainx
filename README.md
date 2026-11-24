# TerrainX

An ESP32 RC car you drive from your phone.

- `TerrainX_BLE/`: firmware for driving over Bluetooth Low Energy (Web Bluetooth controller app).
- `TerrainX_WiFi/`: firmware that makes its own Wi-Fi hotspot "TerrainX-Car". Join it, open http://192.168.4.1 and
  turn the phone sideways.
- `preview/`: the phone controller page, which you can open in a browser to try it.
- `AI_Studio_BLE_App_Prompt.md`: the full spec used to generate the Bluetooth controller app (React + TypeScript).

Flash a sketch with the Arduino IDE (ESP32 board package). Change the hotspot password in the sketch before use.
