For the Pi Zero 2W Submarine Controller

Install Raspberry Pi OS Lite (32bit)

Connect to Pi via SSH

Update:
```
sudo apt update
sudo apt upgrade
```

Install git:
```
sudo apt install -y git
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
