package physac

import "fmt"

const (
	Epsilon float64 = 0.000001
	K       float64 = 1.0 / 3
)

// Check if values are between bias range.
func biasGreaterThan(a, b float64) bool {
	return a >= (b*0.95 + a*0.01)
}

// A debug logger function.
type Debugfn func(string, ...interface{})

// A basic Debugfn that uses `fmt.Printf`.
var FmtDebugfn Debugfn = func(f string, as ...interface{}) {
	fmt.Printf("[PHYSAC] "+f, as...)
}
