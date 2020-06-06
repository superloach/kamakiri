package physac

type Vertex struct {
	Position XY
	Normal   XY
}

// Creates a random polygon shape with max vertex distance from polygon pivot.
func newRandomVertices(radius float64, sides int) []*Vertex {
	panic("stub")
}

// Creates a rectangle polygon shape based on a min and max positions.
func newRectangleVertices(pos, size XY) []*Vertex {
	panic("stub")
}
