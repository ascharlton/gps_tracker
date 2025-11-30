#!/bin/bash
#
usb1='2341:0043'



/usr/sbin/usbip bind --$(/usr/sbin/usbip list -p -l | grep '#usbid='$usb1'#' | cut '-d#' -f1)




