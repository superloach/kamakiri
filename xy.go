package physac

// XY type
type XY struct {
	X float64
	Y float64
}

// Calculates clipping based on a normal and two faces.
func clip(normal XY, _clip float64, a, b *XY) int {
	sp := 0
	out := [2]XY{*a, *b}

	// Retrieve distances from each endpoint to the line
	distanceA := mathDot(normal, *a) - _clip
	distanceB := mathDot(normal, *b) - _clip

	// If negative (behind plane)
	if distanceA <= 0.0 {
		out[sp] = *a
		sp++
	}

	if distanceB <= 0.0 {
		out[sp] = *b
		sp++
	}

	// If the points are on different sides of the plane
	if (distanceA * distanceB) < 0.0 {
		// Push intersection point
		alpha := distanceA / (distanceA - distanceB)
		out[sp] = *a
		delta := xySubtract(*b, *a)
		delta.X *= alpha
		delta.Y *= alpha
		out[sp] = xyAdd(out[sp], delta)
		sp++
	}

	// Assign the new converted values
	*a = out[0]
	*b = out[1]

	return sp
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
	return XY{v1.X + v2.X, v1.Y + v2.Y}
}

// Returns the subtract of two given vectors.
func xySubtract(v1, v2 XY) XY {
	panic("stub")
}
