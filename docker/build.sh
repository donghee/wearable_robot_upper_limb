sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
#docker pull arm64v8/ros:humble
sudo usermod -aG docker $USER

cd "$(dirname "$0")/.."

# If the machine is x86_64, build for arm64
if [ "$(uname -m)" = "x86_64" ]; then
  docker run --privileged --rm tonistiigi/binfmt:qemu-v7.0.0-28 --install arm64,riscv64,arm
  docker build --platform linux/arm64 -t ghcr.io/donghee/wearable_robot_upper_limb:latest -f docker/Dockerfile .
  exit 0
fi

# build in raspberry pi 
docker build -t ghcr.io/donghee/wearable_robot_upper_limb:latest -f docker/Dockerfile .
