package main

import "github.com/superloach/physac-go"

func main() {
	w := physac.NewWorld()

	ball := physac.NewBodyCircle(w, physac.XY{0, 0}, 5, 1, 10)

	ball.AddForce(physac.XY{2, -2})

	for i := 0; i < 10; i++ {
		w.RunStep(w.DeltaTime * 10)
		w.Debug("%8.4f, %8.4f\n", ball.Position.X, ball.Position.Y)
	}
}
