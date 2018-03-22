batchisp -device at32uc3a0512 -hardware usb -operation erase f memory flash blankcheck loadbuffer "Proto2.hex" program verify start reset 0
