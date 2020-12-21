//+build examples

package main

import (
	"fmt"
	"image"
	"image/color"
	"math/rand"

	"github.com/hajimehoshi/ebiten"
	"github.com/hajimehoshi/ebiten/ebitenutil"
	"github.com/hajimehoshi/ebiten/inpututil"

	"github.com/superloach/kamakiri"
)

var circle = func() *ebiten.Image {
	img := image.NewNRGBA(image.Rect(0, 0, 50, 50))

	for r := 25; r > 22; r-- {
		x0, y0, c := 25, 25, color.White

		x, y, dx, dy := r-1, 0, 1, 1
		e := dx - (r * 2)

		for x > y {
			img.Set(x0+x, y0+y, c)
			img.Set(x0+y, y0+x, c)
			img.Set(x0-y, y0+x, c)
			img.Set(x0-x, y0+y, c)
			img.Set(x0-x, y0-y, c)
			img.Set(x0-y, y0-x, c)
			img.Set(x0+y, y0-x, c)
			img.Set(x0+x, y0-y, c)

			if e <= 0 {
				y++
				e += dy
				dy += 2
			}
			if e > 0 {
				x--
				dx += 2
				e += dx - (r * 2)
			}
		}
	}

	eimg, err := ebiten.NewImageFromImage(img, ebiten.FilterDefault)
	if err != nil {
		panic(err)
	}

	return eimg
}()

var green = color.RGBA{0, 255, 0, 255}

const (
	Width  = 800
	Height = 450

	Title = "[kamakiri] basic demo"

	DebugFmt = `%.2f FPS/%.2f TPS
%d circles/%d polygons

left mouse button to create a polygon
right mouse button to create a circle

powered by kamakiri <3`
)

type Game struct {
	World *kamakiri.World
}

func NewGame() *Game {
	game := &Game{
		World: kamakiri.NewWorld(),
	}

	// Create floor rectangle physics body
	floor := game.World.NewBodyRectangle(kamakiri.XY{Width / 2, 0}, 500, 100, 10)
	floor.Enabled = false // Disable body state to convert it to static (no dynamics, but collisions)

	// Create obstacle circle physics body
	circle := game.World.NewBodyCircle(kamakiri.XY{Width / 2, Height / 2}, 45, 10, 8)
	circle.Enabled = false // Disable body state to convert it to static (no dynamics, but collisions)

	return game
}

func (g *Game) Update(_ *ebiten.Image) error {
	g.World.PhysicsStep()

	switch {
	case inpututil.IsMouseButtonJustReleased(ebiten.MouseButtonLeft):
		cx, cy := ebiten.CursorPosition()
		_ = g.World.NewBodyPolygon(
			kamakiri.XY{float64(cx), Height - float64(cy)}, // at cursor position
			rand.Float64()*60+20,                           // random radius in [20, 80)
			rand.Intn(5)+3,                                 // random sides in [3, 8)
			10,                                             // density 10
		)

	case inpututil.IsMouseButtonJustReleased(ebiten.MouseButtonRight):
		cx, cy := ebiten.CursorPosition()
		_ = g.World.NewBodyCircle(
			kamakiri.XY{float64(cx), Height - float64(cy)}, // at cursor position
			rand.Float64()*35+10,                           // random radius in [10, 45)
			10,                                             // density 10
			8,                                              // vertices 8
		)
	}

	for _, body := range g.World.Bodies {
		if body != nil && body.Position.Y < -Height {
			body.Destroy()
		}
	}

	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	circles, polygons := 0, 0

	for _, body := range g.World.Bodies {
		if body == nil {
			continue
		}

		switch body.Shape.Type {
		case kamakiri.ShapeTypePolygon:
			polygons++

			for i, vertexA := range body.Shape.Vertices {
				j := (i + 1) % len(body.Shape.Vertices)
				vertexB := body.Shape.Vertices[j]

				ebitenutil.DrawLine(
					screen,
					body.Position.X+vertexA.Position.X,
					Height-(body.Position.Y+vertexA.Position.Y),
					body.Position.X+vertexB.Position.X,
					Height-(body.Position.Y+vertexB.Position.Y),
					color.White,
				)
			}

		case kamakiri.ShapeTypeCircle:
			circles++

			dio := &ebiten.DrawImageOptions{}
			dio.GeoM.Scale(
				body.Shape.Radius/25,
				body.Shape.Radius/25,
			)
			dio.GeoM.Translate(
				body.Position.X-body.Shape.Radius,
				Height-(body.Position.Y+body.Shape.Radius),
			)

			screen.DrawImage(circle, dio)
		}
	}

	ebitenutil.DebugPrint(screen, fmt.Sprintf(
		DebugFmt,
		ebiten.CurrentTPS(),
		ebiten.CurrentFPS(),
		circles, polygons,
	))
}

func (g *Game) Layout(ow, oh int) (int, int) {
	return Width, Height
}

func main() {
	game := NewGame()

	ebiten.SetWindowSize(Width, Height)

	err := ebiten.RunGame(game)
	if err != nil {
		panic(err)
	}
}
