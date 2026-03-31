For the Pi Zero 2W Submarine Controller

Install Raspberry Pi OS Lite (64bit) Trixie

From the terminal use this command to Connect to Pi via SSH
```
ssh [user]@[hostname]
```

Example:
```
ssh pi@anglerfish.local
```

Enter the config section:
```
sudo raspi-config
```
Go to Interface Options, I2C, Enable.
Update

Install git:
```
sudo apt install -y git
```
Install pigpio
```
sudo apt install -y python3-setuptools python3-full
wget https://github.com/joan2937/pigpio/archive/refs/tags/v79.tar.gz
tar zxf v79.tar.gz
cd pigpio-79
make
sudo make install
sudo ldconfig
sudo systemctl daemon-reload
```
Use "sudo pigpiod" to start pigpio service for troubleshooting. The program will automatically start this service.  
Exit the directory to return to the home directory:
```
cd ..
```

Install MediaMTX:
```
mkdir -p ~/mediamtx
cd ~/mediamtx
wget https://github.com/bluenviron/mediamtx/releases/download/v1.17.0/mediamtx_v1.17.0_linux_arm64.tar.gz
tar -xzf mediamtx_v1.17.0_linux_arm64.tar.gz
chmod +x mediamtx
```
Create Config File:
```
rm mediamtx.yml
nano ~/mediamtx/mediamtx.yml
```
Paste in the new mediamtx.yml file:
```
logLevel: info

# Disable unused stuff (reduces errors + CPU)
rtmp: no
hls: no
webrtc: no
srt: no

paths:
  cam:
    source: publisher
```
Exit the directory to return to the home directory:
```
cd ..
```

Install Python Packages:
```
sudo apt install -y  python3-picamera2 python3-smbus python3-pip
sudo pip3 install adafruit-circuitpython-ads1x15 --break-system-packages
```