sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
#docker pull arm64v8/ros:humble
sudo usermod -aG docker $USER

cd "$(dirname "$0")/.."

docker build -t ghcr.io/donghee/wearable_robot_upper_limb:latest -f docker/Dockerfile .
