package physac

type ShapeType uint8

const (
	ShapeTypeCircle ShapeType = iota
	ShapeTypePolygon
)

type Shape struct {
	Type      ShapeType // Physics shape type (circle or polygon).
	Body      *Body     // Shape physics body reference.
	Radius    float64   // Circle shape radius (used for circle shapes).
	Transform Mat2      // Vertices transform matrix 2x2.
	Vertices  []*Vertex // Polygon shape vertices position and normals data (just used for polygon shapes).
}

// Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON).
func GetShapeType(index int) ShapeType {
	panic("stub")
}

// Finds polygon shapes axis least penetration.
func findAxisLeastPenetration(faceIndex *int, a, b *Shape) float64 {
	panic("stub")
}

// Finds two polygon shapes incident face.
func findIncidentFace(v0, v1 *XY, ref, inc *Shape, index int) {
	panic("stub")
}
