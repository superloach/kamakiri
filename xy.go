package kamakiri

import "math"

// XY is a physics point.
type XY struct {
	X float64
	Y float64
}

// Clip calculates clipping based on the normal v and two faces.
func (v XY) Clip(clip float64, a, b XY) (XY, XY, int) {
	sp := 0
	out := [2]XY{a, b}

	// Retrieve distances from each endpoint to the line
	distanceA := v.Dot(a) - clip
	distanceB := v.Dot(b) - clip

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

// TriangleBarycenter returns the barycenter of a triangle given by v, v2, and v3.
func TriangleBarycenter(v, v2, v3 XY) XY {
	return XY{
		(v.X + v2.X + v3.X) / 3,
		(v.Y + v2.Y + v3.Y) / 3,
	}
}

// Cross returns the cross product of v and v2.
func (v XY) Cross(v2 float64) XY {
	return XY{-v2 * v.Y, v2 * v.X}
}

// CrossXY returns the cross product of v and v2.
func (v XY) CrossXY(v2 XY) float64 {
	return v.X*v2.Y - v.Y*v2.X
}

// LenSqr returns the len square root of a v.
func (v XY) LenSqr() float64 {
	return v.X*v.X + v.Y*v.Y
}

// Dot returns the dot product of v and v2.
func (v XY) Dot(v2 XY) float64 {
	return v.X*v2.X + v.Y*v2.Y
}

// DistSqr returns the square root of the distance between v and v2.
func (v XY) DistSqr(v2 XY) float64 {
	dir := v.Subtract(v2)

	return dir.Dot(dir)
}

// Normalize returns the normalized values of v.
func (v XY) Normalize() XY {
	length := math.Sqrt(v.X*v.X + v.Y*v.Y)

	if length == 0 {
		length = 1
	}

	ilength := 1 / length

	return XY{v.X * ilength, v.Y * ilength}
}

// Add the sum of v and v2.
func (v XY) Add(v2 XY) XY {
	return XY{v.X + v2.X, v.Y + v2.Y}
}

// Subtract returns the difference between v and v2.
func (v XY) Subtract(v2 XY) XY {
	return XY{v.X - v2.X, v.Y - v2.Y}
}
