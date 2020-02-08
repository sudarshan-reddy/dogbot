package bot

import (
	"errors"
	"fmt"
	"image/color"
	"io"
	"os/exec"
	"strconv"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
)

const (
	frameX = 400
	frameY = 300
	port   = "8888"
)

// Follower follows your face
type Follower struct {
	ffmpegIn   io.WriteCloser
	ffmpegOut  io.ReadCloser
	drone      *tello.Driver
	flightData *tello.FlightData
	net        gocv.Net
	green      color.RGBA
	protoPath  string
	modelPath  string
}

// NewFollower creates a new instance of Follower.
func NewFollower(protoPath, modelPath string) (*Follower, error) {

	ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, err := ffmpeg.StdinPipe()
	if err != nil {
		return nil, err
	}
	ffmpegOut, err := ffmpeg.StdoutPipe()
	if err != nil {
		return nil, err
	}

	net := gocv.ReadNetFromCaffe(protoPath, modelPath)
	if net.Empty() {
		return nil, errors.New("error reading network model")
	}
	defer net.Close()

	f := &Follower{
		ffmpegIn:  ffmpegIn,
		ffmpegOut: ffmpegOut,
		drone:     tello.NewDriver(port),
		net:       net,
		protoPath: protoPath,
		modelPath: modelPath,
		green:     color.RGBA{0, 255, 0, 0},
	}

	if err := f.init(ffmpeg); err != nil {
		return nil, err
	}

	return f, nil
}

func (f *Follower) init(cmd *exec.Cmd) error {
	if err := cmd.Start(); err != nil {
		return err
	}

	f.drone.On(tello.FlightDataEvent, func(data interface{}) {
		f.flightData = data.(*tello.FlightData)
	})

	f.drone.On(tello.ConnectedEvent, func(data interface{}) {
		fmt.Println("Connected")
		f.drone.StartVideo()
		f.drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
		f.drone.SetExposure(0)
		gobot.Every(100*time.Millisecond, func() {
			f.drone.StartVideo()
		})
	})

	f.drone.On(tello.VideoFrameEvent, func(data interface{}) {
		pkt := data.([]byte)
		if _, err := f.ffmpegIn.Write(pkt); err != nil {
			fmt.Println(err)
		}
	})

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{f.drone},
	)

	go func() {
		robot.Start()
	}()

	return nil
}
