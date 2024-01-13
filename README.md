## S2 macropad
# Purpose
This is a simple program for a 3x3 macro pad using mechanical keyboard switches. Implemented debouncing and 9 key rollover.

# How to use
Written in ESP-IDF 5.0. Requires tinyusb component. You should set the TinyUSB HID interfaces count parameter in menuconfig to 1. Otherwise upload as normal, after resetting the microcontroller should appear as a keyboard/mouse interface. Normally open switches should be connected between pins and ground (pins are pulled up internally). No extra electronics required.
