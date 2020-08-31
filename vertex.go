package kamakiri

import "math"

// Vertex is a part of a polygon.
type Vertex struct {
	Position XY
	Normal   XY
}

// Creates a random polygon shape with max vertex distance from polygon pivot.
func newRandomVertices(radius float64, sides int) []Vertex {
	poly := make([]Vertex, sides)

	// Calculate polygon vertices positions
	for i := 0; i < sides; i++ {
		poly[i].Position = XY{
			math.Cos(360.0/float64(sides*i)*deg2Rad) * radius,
			math.Sin(360.0/float64(sides*i)*deg2Rad) * radius,
		}
	}

	// Calculate polygon faces normals
	for i := 0; i < sides; i++ {
		nextIndex := (i + 1) % sides
		face := poly[nextIndex].Position.Subtract(poly[i].Position)
		poly[i].Normal = (XY{face.Y, -face.X}).Normalize()
	}

	return poly
}

// Creates a rectangle polygon shape based on a min and max positions.
func newRectangleVertices(pos, size XY) []Vertex {
	poly := make([]Vertex, 4)

	// Calculate polygon vertices positions
	poly[0].Position = XY{pos.X + size.X/2, pos.Y - size.Y/2}
	poly[1].Position = XY{pos.X + size.X/2, pos.Y + size.Y/2}
	poly[2].Position = XY{pos.X - size.X/2, pos.Y + size.Y/2}
	poly[3].Position = XY{pos.X - size.X/2, pos.Y - size.Y/2}

	// Calculate polygon faces normals
	for i := 0; i < 4; i++ {
		nextIndex := (i + 1) % 4
		face := poly[nextIndex].Position.Subtract(poly[i].Position)

		poly[i].Normal = (XY{face.Y, -face.X}).Normalize()
	}

	return poly
}
