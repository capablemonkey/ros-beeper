# beeper

Listens for velocity commands and plays beeping sounds to indicate motion.  Has a configurable interval, so that a sound is played no more often than every x seconds.

TODO: generalize to any other topic
TODO: allow a random sound to be played from a set of sounds

## Install

Requires `sound_play`, which is included in `audio_common`.  Here's how to install `audio_common` for ROS Indigo on Ubuntu:

```
sudo apt-get install ros-indigo-audio-common
```


## Running

Start sound_play node:

```
rosrun sound_play soundplay_node.py
```

Then run this script, which will start the `beeper` node.

```
python main.py
```

## Configuration

You can specify the sound file (WAV or OGG) and the interval between beeps (in seconds) by changing the constants at the top of `main.py`.  TODO: figure out how to use rosparam for this