#ESP-NOW example (esp32S3-DevkitC-1)

this example project is modified o work with the newer esp32-S3-DevKitC-1 board.
it uses ESP-IDF: 5.2.1 and ESP-NOW component: 2.5.0

the major issue with the previous version of the example project is that the plugins for VSCode do not work proprely nad some parameters need to be fixed manualy.
My example fixes the stack overflow issue witihin the example_espnow_task, by increasing the task stack from 2048 bytes to 8048 bytes.

This is a small improvement intended as a starting point for other ESPNOW projects 
