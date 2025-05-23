#!/bin/sh
pip3 install Adafruit_PCA9685
i2cdetect -y -r 1
# vi ~/.local/lib/python3.6/site-packages/Adafruit_GPIO/I2C.py

