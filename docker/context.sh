# set docker context for raspberry pi
docker context create pi --description "Docker host in Raspberry Pi 5" --docker "host=ssh://pi.lan"
docker context ls
docker context use pi
docker context ls

# reset docker context
#docker context use default; docker context rm pi
