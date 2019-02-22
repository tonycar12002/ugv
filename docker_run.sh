xhost +local:root
if [ $# -gt 0 ]; then
	if [ "$1" == "n" ]; then
		nvidia-docker run --name subt_gazebo --rm -it --net=host --privileged -v /dev:/dev \
			-v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
			--env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" argnctu/subt:latest
	else
		docker exec -it subt_gazebo bash 
	fi
else
	echo "please provide docker tag name."
fi
	


