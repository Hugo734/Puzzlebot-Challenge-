sudo docker run -it --rm \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/jjj/Puzzlebot-Challenge-/shared:/puzzlebot_ws/shared \
  puzzlebot bash
