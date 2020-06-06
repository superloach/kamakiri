package physac

import "math"

// Mat2 type (used for polygon shape rotation matrix).
type Mat2 struct {
	M00 float64
	M01 float64
	M10 float64
	M11 float64
}

// Creates a matrix 2x2 from a given radians value.
func mat2Radians(radians float64) Mat2 {
	c := math.Cos(radians)
	s := math.Sin(radians)
	return Mat2{c, -s, s, c}
}

// Set values from radians to a created matrix 2x2.
func mat2Set(matrix *Mat2, radians float64) {
	cos := math.Cos(radians)
	sin := math.Sin(radians)

	matrix.M00 = cos
	matrix.M01 = -sin
	matrix.M10 = sin
	matrix.M11 = cos
}

// Returns the transpose of a given matrix 2x2.
func mat2Transpose(matrix Mat2) Mat2 {
	return Mat2{
		matrix.M00, matrix.M10,
		matrix.M01, matrix.M11,
	}
}

// Multiplies a vector by a matrix 2x2
func mat2MultiplyXY(matrix Mat2, vector XY) XY {
	return XY{
		matrix.M00*vector.X + matrix.M01*vector.Y,
		matrix.M10*vector.X + matrix.M11*vector.Y,
	}
}
