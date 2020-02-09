# dogbot
Code for the DJI tello to follow your face

![drone demo](demo/drone.gif)

# Prequisites:
 - opencv
 - gocv
 - ffmpeg

# Build
 
 - Follow the instructions to install GOCV here: https://github.com/hybridgroup/gocv
 - go build after should work
 - the model and proto.txt are hardcoded at the moment to stay in the same root as the binary.

# Running 

 - Connect your computer with the code to the tello's wifi (should be TELLO-XXXXXX).
 - go run main.go or download the built binary for mac in releases.
 - hit U to have the drone take-off
 - hit T to start tracking mode which follows your face.
 - T to toggle it off
 - D to bring the drone down again.
