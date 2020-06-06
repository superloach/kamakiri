package physac

import "fmt"

const (
	Epsilon float64 = 0.000001
	K       float64 = 1.0 / 3
)

// Check if values are between bias range.
func biasGreaterThan(a, b float64) bool {
	panic("stub")
}

type Debugfn func(string, ...interface{})

var FmtDebugfn Debugfn = func(f string, as ...interface{}) {
	fmt.Printf("[PHYSAC] "+f, as...)
}
