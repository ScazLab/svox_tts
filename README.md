# Speech

A ROS wrapper for the svox-pico TTS Engine. Based on the analogous YARP wrapper: https://github.com/robotology/speech/tree/master/svox-speech , made by @apaikan.

## Requirements

The current implementation requires Linux **`aplay`** command (in the `alsa-utils` package) to play the generated wave file.


## Installation

Compile and install the software with your favorite `catkin` build tool.


## Testing

### First Terminal

```sh
roslaunch svox_tts svox_tts.launch
```

### Second Terminal

The `svox_tts` node exposes a service reachable at `/svox_tts/speech` with an interface that can set some internal parameters (pitch, speed, language) as well as say some sentences. Here are some examples:

```sh
rosservice call /svox_tts/speech "{mode: 5, string: 'We, robots, love you.'}"
rosservice call /svox_tts/speech "{mode: 2, value: 60}"
rosservice call /svox_tts/speech "{mode: 3, value: 80}"
rosservice call /svox_tts/speech "{mode: 5, string: 'Although we should improve our TTS capabilities.'}"
rosservice call /svox_tts/speech "{mode: 1, string: 'it-IT'}"
rosservice call /svox_tts/speech "{mode: 2, value: 100}"
rosservice call /svox_tts/speech "{mode: 5, string: 'Noi, robots, vi amiamo. Anche se dovremmo migliorare le nostre capacità nel TTS.'}"
rosservice call /svox_tts/speech "{mode: 6}"
rosservice call /svox_tts/speech "{mode: 5, string: 'At least now we can speak multiple languages.'}"
rosservice call /svox_tts/speech "{mode: 5, string: 'Here is a list of the languages we can speak:'}"
rosservice call /svox_tts/speech "{mode: 4}"
rosservice call /svox_tts/speech "{mode: 1, string: 'fr-FR'}"
rosservice call /svox_tts/speech "{mode: 5, string: 'Adieu.'}"
```
