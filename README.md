# Scout MQTT Client Setup

This guide explains how to set up the Scout robot

## First SSH into the Moorebot and gaining Root Access

Use the following credentials to connect:

```bash
ssh linaro@<Moorebot_IP>
# Password: linaro
```
Get root access to the scout by adding "chmod 4755 /usr/bin/sudo" before the exit 0 line in the "/etc/rc.local" file.

```bash
vim /etc/rc.local
```
to ennter into the file and then edit the file by pressing "i" and add "chmod 4755 /usr/bin/sudo" before the first line. Press esc to exit out of edit mode and do

```bash
:wq!
```
Reboot the scout by either typing reboot on the terminal or powercycle and now there is root access on the scout. 

## Second get MQTT Running

Once you ssh and gain root access, switch to a root user by 

```bash
sudo su -
```
This puts you into the root-level userspace. 

Create a new directory called scout-files and all of the scripts will be placed in here: 

```bash
mkdir scout-files
cd scout-files/
```
