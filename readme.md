# ablibs-eeprom

## Introduction

Classes for interfacing to common flash memory chips.

The following devices are supported:

* 24LC1026 I2C device

## Coming soon

* Support for other devices.

## 24C1026

This device is 1MB organised as 128K x 8 bytes with 128 bytes per page. As
the library uses a flexible method of reading and writing to the device other
similar device albeit with different memory sizes may be already supported, just
not tested.
