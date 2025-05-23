# /bin/sh
#### SET PWM ####
# assign Pin32 to PWM0
busybox devmem 0x700031fc 32 0x45
busybox devmem 0x6000d504 32 0x2
# assign Pin33 to PWM2
busybox devmem 0x70003248 32 0x46
busybox devmem 0x6000d100 32 0x00

cd /sys/devices/7000a000.pwm/pwm/pwmchip0
# control Pin 32
echo 0 > export
echo 20000000 > pwm0/period
echo 2500000 > pwm0/duty_cycle
echo 1 > pwm0/enable

# Control Pin33
echo 2 > export
echo 20000000 > pwm2/period
echo 1500000 > pwm2/duty_cycle
echo 1 > pwm2/enable
