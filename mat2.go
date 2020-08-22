package physac

import "math"

// Mat2 type (used for polygon shape rotation matrix).
type Mat2 [2][2]float64

// Creates a matrix 2x2 from a given radians value.
func Mat2Radians(radians float64) Mat2 {
	c := math.Cos(radians)
	s := math.Sin(radians)
	return Mat2{
		{c, -s},
		{s, c},
	}
}

// Returns the transpose of a given matrix 2x2.
func (m Mat2) Transpose() Mat2 {
	return Mat2{
		{m[0][0], m[1][0]},
		{m[0][1], m[1][1]},
	}
}

// Multiplies a vector by a matrix 2x2
func (m Mat2) MultiplyXY(v XY) XY {
	return XY{
		m[0][0]*v.X + m[0][1]*v.Y,
		m[1][0]*v.X + m[1][1]*v.Y,
	}
}
