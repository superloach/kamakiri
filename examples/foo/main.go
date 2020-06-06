package main

import (
	"fmt"

	"github.com/superloach/physac-go"
)

func main() {
	w := physac.NewWorld()

	ball := physac.NewBodyCircle(w, physac.XY{0, 0}, 5, 1, 10)

	for i := 0; i < 10; i++ {
		w.RunStep()
		fmt.Println(ball)
	}
}
