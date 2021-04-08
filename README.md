# MultiBoot

## Simple arduino fimware programmer

Save your .hex files on SDcard, select one in menu with the buttons Up and Down, then press Ok to program the controller.

Supported controllers for program: Atmega 328p

For reboot programmer send `reboot` from controller.
For load another file send `load:filename.hex`

Programmer <-> Controller

D3 --------- SCK

D5 --------- Reset

D6 --------- MISO

D7 --------- MOSI

Programmer <-> SD        

D4  ------- CS

D11 ------- MOSI

D12 ------- MISO

D13 ------- SCK

Programmer <- Buttons

A0 -------- Up

A1 -------- Down

A2 -------- Ok

D8 -------- Cancel


Projects: 
Simple game console: https://easyeda.com/Forem/multi
