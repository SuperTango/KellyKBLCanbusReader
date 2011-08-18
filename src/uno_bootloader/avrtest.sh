#
# for UNO
#
/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avrdude \
        -C/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf \
        -patmega328p \
        -cstk500v1 \
        -P /dev/tty.usbmodem1a21 \
        -b115200 \
        -v

# #
# # for mega
# #
# /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avrdude \
#         -C/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf \
#         -patmega2560 \
#         -cstk500v2 \
#         -P /dev/tty.usbmodem1a21 \
#         -b115200 \
#         -v
# 
