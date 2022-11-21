# System Start Procedure

### One Time Personal Computer Setup (Static Ip)
1. Navigate to network in settings
2. Select Ethernet connection and navigate to IPV4
3. Set Method to manual 
4. Enter an address of:  `192.168.10.*` (10.10 is reserved for the raspberry pi)
5. Set netmask to `255.255.255.0`

# Main opertations

## Pi startup
1. Plug raspberry pi into battery pack
2. Wait ~1-2 minutes to let boot
3. Plug ethernet cable into raspberry pi

## Wired SSH
1. Plug ethernet cable into laptop
2. Open a terminal and type ```ssh ubuntu@192.168.10.10 ```
3. If it asks to trust the new connection enter yes
4. Enter the password for ubuntu
5. Your terminal should now be `ubuntu@ubuntu`

## Hotspot Startup
1. Type the command `sudo nmcli d wifi hotspot ifname wlan0 ssid TrakBot password <pasword for ubuntu>`
2. Wait for the message:  ```Device 'wlan0' successfully activated with '137d39a2-4b59-45c0-bba9-61a8f26c8e10'.
Hint: "nmcli dev wifi show-password" shows the Wi-Fi name and password.```

## Connect iphone
1. Plug iphone in via USB
2. Type `sudo ifconfig`
3. Confirm that the inet of eth1 is `172.20.10.5`

## Wireless SSH
1. Unplug ethernet cable from pi
2. Connect to TrakBot wifi (note you will no longer have internet access)
3. In a new terminal type `ssh ubuntu@10.42.0.1`
3. If it asks to trust the new connection enter yes
4. Enter the password for ubuntu
5. Your terminal should now be `ubuntu@ubuntu`

## Shutdown procedure 
1. Enter the command `sudo shutdown`
2. Wait ~2-4 minutes before unplugging the raspberry pi

# Helpful Debugging Tools

## Toggle Graphical Interface
1. To turn on graphical interface for pi type `systemctl set-default graphical.target`
2. Reboot to apply changes
3. To turn off graphical interface for pi type `systemctl set-default multi-user.target`
4. Reboot to apply changes