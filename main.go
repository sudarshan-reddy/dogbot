package main

import (
	"github.com/sudarshan-reddy/dogbot/bot"
	"gocv.io/x/gocv"
)

func main() {
	followerBot, err := bot.NewFollower("./proto.txt", "./model")
	if err != nil {
		panic(err)
	}

	var window = gocv.NewWindow("Tello")
	if err := followerBot.Start(window); err != nil {
		panic(err)
	}

}
