XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH
docker run --runtime=nvidia \
           --rm \
	   --gpus all \
	   -it \
	   --privileged \
	   --network=host \
	   -e NVIDIA_DRIVER_CAPABILITIES=all \
	   -e DISPLAY=$DISPLAY \
	   -v $XSOCK:$XSOCK \
	   -v $XAUTH:$XAUTH \
	   -e XAUTHORITY=$XAUTH \
	   --volume "./visualisation/config.yaml:/src/visualisation/config.yaml" \
	   --volume "./visualisation/data/rosbags:/src/visualisation/data/rosbags" \
	   --volume "./visualisation/data/zarrfiles:/src/visualisation/data/zarrfiles" \
	   rerun:0.30.0
