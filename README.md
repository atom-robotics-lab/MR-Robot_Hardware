# MR_Robot_Firmware
First of all flash your pi with the image provided by atom.It is done using raspberry imager (available on ubuntu software as well).Then remove sd-card and re-insert it and go to rootfs/etc/wpa_supplicant and run command
```bash
sudo nano wpa_supplicant.config
```
Add your WIFI and password over there and press ctrl X ,then press Y anfd then enter.
Now go to boot and a .ssh file over there using command.
```bash
touch .ssh
```
Now insert your sd-card into the pi now u are ready to go.

# PI
Now to navigate inside the pi.Type the command
```bash
ssh pi@I.P Address of the pi
```

Download a application called FING to check out the Ip address of nearby devices.

After following the above command the pi will ask for a password which is:
```bash
raspberry
```
After that navigate into Ros_workspaces and create a workspace catkin_ws(u can always check wiki-ros if u get stuck at this step :) )

Now clone the repo MR-ROBOT_Hardware using command
```bash
git clone git@github.com:atom-robotics-lab/MR-Robot_Hardware.git
```
# KEEP SOURCING YOUR WORKSPACE

# I.P SETUP
