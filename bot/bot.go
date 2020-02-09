package bot

import (
	"errors"
	"fmt"
	"image"
	"image/color"
	"io"
	"math"
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

// Start the drone.
// Not threadsafe at all. Doesn't have to be.
func (f *Follower) Start() {
	frameSize := frameX * frameY * 3
	refDistance := float64(0)
	detected := false
	left := float32(0)
	top := float32(0)
	right := float32(0)
	bottom := float32(0)
	var window = gocv.NewWindow("Tello")
	var tracking = false
	var detectSize = false
	var distTolerance = 0.05 * dist(0, 0, frameX, frameY)

	for {
		outData := make([]byte, frameSize)
		if _, err := io.ReadFull(f.ffmpegOut, outData); err != nil {
			fmt.Println(err)
			continue
		}
		img, err := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, outData)
		if err != nil {
			fmt.Println(err)
			continue
		}
		if img.Empty() {
			continue
		}
		W := float32(img.Cols())
		H := float32(img.Rows())

		blob := gocv.BlobFromImage(img, 1.0, image.Pt(128, 96), gocv.NewScalar(104.0, 177.0, 123.0, 0), false, false)
		defer blob.Close()

		f.net.SetInput(blob, "data")

		detBlob := f.net.Forward("detection_out")
		defer detBlob.Close()
		detections := gocv.GetBlobChannel(detBlob, 0, 0)
		defer detections.Close()

		for r := 0; r < detections.Rows(); r++ {
			confidence := detections.GetFloatAt(r, 2)
			if confidence < 0.5 {
				continue
			}

			left = detections.GetFloatAt(r, 3) * W
			top = detections.GetFloatAt(r, 4) * H
			right = detections.GetFloatAt(r, 5) * W
			bottom = detections.GetFloatAt(r, 6) * H

			left = min(max(0, left), W-1)
			right = min(max(0, right), W-1)
			bottom = min(max(0, bottom), H-1)
			top = min(max(0, top), H-1)

			rect := image.Rect(int(left), int(top), int(right), int(bottom))
			gocv.Rectangle(&img, rect, f.green, 3)
			detected = true
		}

		window.IMShow(img)
		if window.WaitKey(10) >= 0 {
			break
		}

		if !tracking || !detected {
			continue
		}

		if detectSize {
			detectSize = false
			refDistance = dist(left, top, right, bottom)
		}

		distance := dist(left, top, right, bottom)

		if right < W/2 {
			f.drone.CounterClockwise(50)
		} else if left > W/2 {
			f.drone.Clockwise(50)
		} else {
			f.drone.Clockwise(0)
		}

		if top < H/10 {
			f.drone.Up(25)
		} else if bottom > H-H/10 {
			f.drone.Down(25)
		} else {
			f.drone.Up(0)
		}

		if distance < refDistance-distTolerance {
			f.drone.Forward(20)
		} else if distance > refDistance+distTolerance {
			f.drone.Backward(20)
		} else {
			f.drone.Forward(0)
		}
	}

}

func dist(x1, y1, x2, y2 float32) float64 {
	return math.Sqrt(float64((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)))
}

func min(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func max(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}
