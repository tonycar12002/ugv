xhost +local:root
SAME="false"
docker_target="argnctu/subt:latest"
docker_name="subt_gazebo"

for ARGI; do
    if [ "${ARGI}" = "--same" -o "${ARGI}" = "-s" ] ; then
        SAME="true"
        echo "SAME docker mode ON"   
    fi

	if [ "${ARGI}" = "--subt" -o "${ARGI}" = "-st" ] ; then
        docker_target="argnctu/subt:latest"
		docker_name="subt_gazebo"
         

	elif [ "${ARGI}" = "--ugv" -o "${ARGI}" = "-u" ] ; then
        docker_target="tonycar12002/ugv:latest"
		docker_name="ugv"
        
    fi
done
echo "Target docker",  $docker_target 
echo "Target docker",  $docker_name  

if [ "$SAME" == "true" ]; then
	docker exec -it $docker_name bash
elif [ "$docker_name" == "subt_gazebo" ]; then
	nvidia-docker run --name $docker_name  --rm -it --net=host --privileged -v /dev:/dev \
				-v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
				-v ~/.bashrc:/root/.bashrc \
				-v /home/$USER/SubT/:/home/developer/SubT/ \
				--pid=host \
				--env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" $docker_target
else
	nvidia-docker run --name $docker_name --rm -it --net=host --privileged -v /dev:/dev \
				-v /etc/localtime:/etc/localtime:ro -v /var/run/docker.sock:/var/run/docker.sock \
				-v ~/.bashrc:/root/.bashrc \
				-v /home/$USER/SubT:/root/SubT \
				--env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" $docker_target
fi

