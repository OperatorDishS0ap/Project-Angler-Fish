For the Pi Zero 2W Submarine Controller

Install Raspberry Pi OS Lite (64bit)

From the terminal use this command to find the IP address of the PI
```
ping [hostname]
```
Example:
```
ping anglerfish
```

Connect to Pi via SSH

Update:
```
sudo apt update
sudo apt full-upgrade
```

Install Python Packages:
```
sudo apt install -y python3-paramiko python3-numpy python3-gpiozero python3-picamera2 python3-av python3-smbus
```

Install GStreamer
```
sudo apt install -y \
python3-gi \
python3-gst-1.0 \
gstreamer1.0-tools \
gstreamer1.0-plugins-base \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-libav
sudo apt install gstreamer1.0-plugins-ugly
sudo apt install -y \
gir1.2-gst-rtsp-server-1.0 \
libgstrtspserver-1.0-0
```

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
Execute to start pigpio:
```
sudo pigpiod
```

Add Files:
```
git clone --filter=blob:none --no-checkout https://github.com/OperatorDishS0ap/Project-Angler-Fish.git
cd Project-Angler-Fish
git sparse-checkout init --cone
git sparse-checkout set Pi
git checkout main
```

To Update Files:
```
cd ~/Project-Angler-Fish
git pull
```
Given the files are corrupted or changed on the raspberry pi delete the the folder and re-add the repository files.
You can delete the folder from the home directory with the command:
```
rm -rf Project-Angler-Fish
```
