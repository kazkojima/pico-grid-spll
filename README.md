# pico-grid-spll

An experimentation of the software PLL for power grid frequency with raspberry pi pico. Ported from TI's ["Software PLL Design Using C2000"](https://www.ti.com/lit/pdf/sprabt3).

Pin assign is here:
| AC input(*) | GPIO26 |
|-------------|--------|
| 8fo         | GPIO16 |
| 4fo         | GPIO17 |
| 2fo         | GPIO18 |
|  fo         | GPIO19 |
| /fo         | GPIO20 |
where fo is SPLL output.

The program assumes AC input frequency is 50Hz. It can be changed the line
```
#define GRID_FREQ 50
```
in grid_spll/main.c.
AC input given to ADC CH0 pin (GPIO26) is assumes ~1Vp-p offseted ~2V.
One can set the offset voltage with changing the line
```
#define AC_INPUT_OFFSET 0x981
```
appropriatly. For example, 1.65v offset can be set with
```
#define AC_INPUT_OFFSET 0x800
```

The program uses TI's 32-bit fixed point arithmetic library IQMath in mspm0-sdk which is given as a git submodule.
