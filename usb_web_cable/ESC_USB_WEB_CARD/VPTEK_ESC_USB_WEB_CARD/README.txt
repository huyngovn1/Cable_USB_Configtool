VPTEK CARD FIRMWARE MENU UPDATE

Replace these three files together:
- DisplayUI.cpp
- web_bridge.cpp
- web_bridge.h

Keep builtin_firmware.h in the same Arduino project folder. The included copy
is provided so the built-in VPTEK firmware image is not missing.

Operation:
1. Short press KEY OK on the Home screen: connect and read ESC settings.
2. Edit a setting and press OK: save, verify and restart ESC as before.
3. Hold KEY OK for 1.2 seconds on Home/Menu/Edit: open the Firmware page.
4. Select INSTALL VPTEK FW or EXIT FIRMWARE with Up/Down.
5. Press OK on INSTALL VPTEK FW to start.
6. Wi-Fi AP is stopped before the first firmware write.
7. The LCD shows the real write stage, bytes written and calculated percent.
8. Success is reported only after restoring the EEPROM safety byte and
   restarting the ESC.

Compatibility:
- The built-in image is restricted by the existing project to AT32F421,
  flash code 0x1F.
- Do not remove power while the Firmware page is writing or finalizing.

Customer Car-only interface:
- Card and Web hide Reverse Rotation, Bi-directional, 30 ms Telemetry,
  Signal Type and Car Reverse.
- Their EEPROM offsets and protocol fields are not deleted. The desktop App
  Tool can still read and change all of them for factory/service setup.
