## ROM
Inputs: clk, reset, MDR_bus, load_MDR, load_MAR, CS, R_NW
Inout: sysbus
The program is located in read only memory (ROM).

load_MAR control signal stores data in look up table in FPGA board.

mar | mdr
----|----
00000|store, 10000
00001|load, 10000
00010|add, 00101
00011|store, 10000
00100|bne, 00110
00101|2
00110|1
...|0

The memory send data to sysbus only if the mar[4] is '1' or '0'. For mar[4] = '1' RAM writes data to bus, otherwise ROM does it. ROM use only 7 addresses while RAM use 16.

5 bit address:
1st digit: `1` - RAM writes to bus, `0` ROM writes to bus
Four digits: address of memory.

The high impedance `Z` state of sysbus is shown as `7f` state on display e.g. display is turned off. It's not possible to detect `Z` or `X` states since they are unmeasureable.

Memory|Components
---|---
0-15|ROM
16-29|RAM
30-31|Switch/segment reg
