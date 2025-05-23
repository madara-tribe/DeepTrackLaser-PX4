#/bin/sh
# git clone from source
git clone https://github.com/NVIDIA/jetson-gpio.git
cd jetson-gpio
python3 setup.py install

# create group and add user
groupadd -f -r gpio
usermod -a -G gpio <ユーザー名>

# add udev rule
cp lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
udevadm control --reload-rules && udevadm trigger
reboot
