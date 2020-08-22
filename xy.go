package physac

import "math"

// XY type
type XY struct {
	X float64
	Y float64
}

// Calculates clipping based on a normal and two faces.
func (norm XY) Clip(clip float64, a, b XY) (XY, XY, int) {
	sp := 0
	out := [2]XY{a, b}

	// Retrieve distances from each endpoint to the line
	distanceA := norm.Dot(a) - clip
	distanceB := norm.Dot(b) - clip

	// If negative (behind plane)
	if distanceA <= 0.0 {
		out[sp] = a
		sp++
	}

	if distanceB <= 0.0 {
		out[sp] = b
		sp++
	}

	// If the points are on different sides of the plane
	if (distanceA * distanceB) < 0.0 {
		// Push intersection point
		alpha := distanceA / (distanceA - distanceB)
		out[sp] = a
		delta := b.Subtract(a)
		delta.X *= alpha
		delta.Y *= alpha
		out[sp] = out[sp].Add(delta)
		sp++
	}

	return out[0], out[1], sp
}

// Returns the barycenter of a triangle given by 3 points.
func TriangleBarycenter(v1, v2, v3 XY) XY {
	return XY{
		(v1.X + v2.X + v3.X) / 3,
		(v1.Y + v2.Y + v3.Y) / 3,
	}
}

// Returns the cross product of a vector and a value.
func (v XY) Cross(value float64) XY {
	return XY{
		-value * v.Y,
		value * v.X,
	}
}

// Returns the cross product of two vectors.
func (v1 XY) CrossXY(v2 XY) float64 {
	return v1.X*v2.Y - v1.Y*v2.X
}

// Returns the len square root of a vector.
func (v XY) LenSqr() float64 {
	return v.X*v.X + v.Y*v.Y
}

// Returns the dot product of two vectors.
func (v1 XY) Dot(v2 XY) float64 {
	return v1.X*v2.X + v1.Y*v2.Y
}

// Returns the square root of distance between two vectors.
func (v1 XY) DistSqr(v2 XY) float64 {
	dir := v1.Subtract(v2)
	return dir.Dot(dir)
}

// Returns the normalized values of a vector.
func (v XY) Normalize() XY {
	length := math.Sqrt(v.X*v.X + v.Y*v.Y)

	if length == 0 {
		length = 1
	}

	ilength := 1 / length

	return XY{v.X * ilength, v.Y * ilength}
}

// Returns the sum of two given vectors.
func (v1 XY) Add(v2 XY) XY {
	return XY{v1.X + v2.X, v1.Y + v2.Y}
}

// Returns the subtract of two given vectors.
func (v1 XY) Subtract(v2 XY) XY {
	return XY{v1.X - v2.X, v1.Y - v2.Y}
}
