Speech
=======
A ROS wrapper for the svox-pico TTS Engine. Based on the analogous YARP wrapper: https://github.com/robotology/speech/tree/master/svox-speech , made by @apaikan.


Requirements
------------
The current implementation requires Linux **`aplay`** command (in the `alsa-utils` package) to play the generated wave file.


Installation
------------
Compile and install the software with your favorite `catkin` build tool.


Testing
-------
```sh
rosrun svox_tts svox_tts
```

```sh
rostopic pub --once /svox_tts/speech std_msgs/String "We, robots, love you!"
```


