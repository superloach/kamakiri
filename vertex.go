package physac

import "math"

type Vertex struct {
	Position XY
	Normal   XY
}

// Creates a random polygon shape with max vertex distance from polygon pivot.
func newRandomVertices(radius float64, sides int) []Vertex {
	data := make([]Vertex, sides)

	// Calculate polygon vertices positions
	for i := 0; i < sides; i++ {
		data[i].Position.X = math.Cos(360.0/float64(sides*i)*Deg2Rad) * radius
		data[i].Position.Y = math.Sin(360.0/float64(sides*i)*Deg2Rad) * radius
	}

	// Calculate polygon faces normals
	for i := 0; i < sides; i++ {
		nextIndex := (i + 1) % sides
		face := data[nextIndex].Position.Subtract(data[i].Position)

		data[i].Normal = (XY{face.Y, -face.X}).Normalize()
	}

	return data
}

// Creates a rectangle polygon shape based on a min and max positions.
func newRectangleVertices(pos, size XY) []Vertex {
	data := make([]Vertex, 4)

	// Calculate polygon vertices positions
	data[0].Position = XY{pos.X + size.X/2, pos.Y - size.Y/2}
	data[1].Position = XY{pos.X + size.X/2, pos.Y + size.Y/2}
	data[2].Position = XY{pos.X - size.X/2, pos.Y + size.Y/2}
	data[3].Position = XY{pos.X - size.X/2, pos.Y - size.Y/2}

	// Calculate polygon faces normals
	for i := 0; i < 4; i++ {
		nextIndex := (i + 1) % 4
		face := data[nextIndex].Position.Subtract(data[i].Position)

		data[i].Normal = (XY{face.Y, -face.X}).Normalize()
	}

	return data
}
