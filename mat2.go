package physac

// Mat2 type (used for polygon shape rotation matrix).
type Mat2 struct {
	M00 float64
	M01 float64
	M10 float64
	M11 float64
}

// Creates a matrix 2x2 from a given radians value.
func mat2Radians(radians float64) Mat2 {
	panic("stub")
}

// Set values from radians to a created matrix 2x2.
func mat2Set(matrix *Mat2, radians float64) {
	panic("stub")
}

// Returns the transpose of a given matrix 2x2.
func mat2Transpose(matrix Mat2) Mat2 {
	panic("stub")
}

// Multiplies a vector by a matrix 2x2
func mat2MultiplyXY(matrix Mat2, vector XY) XY {
	panic("stub")
}
