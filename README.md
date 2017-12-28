# beeper

Listens for velocity commands and plays beeping sounds to indicate motion.  Has a configurable interval, so that a sound is played no more often than every x seconds.

TODO:
- generalize to other topics
- support multiple sounds
- play a sound at random
- configurable log level

## Install

Requires `sound_play`, which is included in `audio_common`.  Here's how to install `audio_common` for ROS Indigo on Ubuntu:

```
sudo apt-get install ros-indigo-audio-common
```

Then, clone this repo into your ROS workspace `src` directory. From your ROS workspace root, do:

```
catkin_make
```

## Running

Start sound_play node:

```
rosrun sound_play soundplay_node.py
```

Then start the `beeper` node:

```
rosrun beeper beeper.py
```

## Configuration

You can specify the sound file (WAV or OGG) and the interval between beeps (in seconds) by setting the following parameters via `rosparam`:

```
rosparam set beeper/sound_file /absolute/path/to/file.wav
rosparam set beeper/interval 5
```