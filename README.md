# Simple Python API for ROBOTIS OP3
Intend to easier Robot develop.
## Requires
* Environment setup followed [official e-manual](http://emanual.robotis.com/docs/en/platform/op3/recovery/#ros-installation--environment-setup)
* Modified [ROBOTIS-OP3 package](https://github.com/culdo/ROBOTIS-OP3)
* Modified [ROBOTIS-OP3-Tools package](https://github.com/culdo/ROBOTIS-OP3-Tools)
## Useful API
* `online_walking_command()`
* `play_motion(page_num)`
* `google_tts(speech_text, lang="zh")` OP3 can speak 100+ languages using Google TTS API.
* `safety_suspend()` Go improved initial pose then turning all torque off.
## Simulation
Not Test Yet
