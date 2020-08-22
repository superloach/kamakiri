package main

import (
	"fmt"

	"github.com/superloach/physac-go"
)

func main() {
	w := physac.NewWorld()
	fmt.Printf("made world %v\n", w)

	ball := physac.NewBodyCircle(w, physac.XY{0, 0}, 5, 1, 10)
	fmt.Printf("made ball %v\n", ball)

	force := physac.XY{2, 20}

	fmt.Printf("applying force %v\n", force)
	ball.AddForce(force)

	for i := 0; i < 10; i++ {
		fmt.Printf("ball at %v\n", ball.Position)
		w.RunStep(w.DeltaTime * 10)
	}
}
