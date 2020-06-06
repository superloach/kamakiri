package physac

// XY type
type XY struct {
	X float64
	Y float64
}

// Calculates clipping based on a normal and two faces.
func clip(normal XY, _clip float64, a, b *XY) int {
	panic("stub")
}

// Returns the barycenter of a triangle given by 3 points.
func triangleBarycenter(v1, v2, v3 XY) XY {
	panic("stub")
}

// Returns the cross product of a vector and a value.
func mathCross(value float64, vector XY) XY {
	panic("stub")
}

// Returns the cross product of two vectors.
func mathCrossXY(v1, v2 XY) float64 {
	panic("stub")
}

// Returns the len square root of a vector.
func mathLenSqr(vector XY) float64 {
	panic("stub")
}

// Returns the dot product of two vectors.
func mathDot(v1, v2 XY) float64 {
	panic("stub")
}

// Returns the square root of distance between two vectors.
func distSqr(v1, v2 XY) float64 {
	panic("stub")
}

// Returns the normalized values of a vector.
func mathNormalize(vector *XY) {
	panic("stub")
}

// Returns the sum of two given vectors.
func xyAdd(v1, v2 XY) XY {
	panic("stub")
}

// Returns the subtract of two given vectors.
func xySubtract(v1, v2 XY) XY {
	panic("stub")
}
