sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
docker pull arm64v8/ros:humble
sudo usermod -aG docker $USER

