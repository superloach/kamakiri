package physac

// Abstraction of globals from original Physac lib.
type World struct {
	CollisionIterations   float64
	PenetrationAllowance  float64
	PenetrationCorrection float64

	BaseTime    float64
	StartTime   float64
	DeltaTime   float64
	CurrentTime float64
	Frequency   uint64

	Accumulator  float64
	StepsCount   uint
	GravityForce XY
	Bodies       []*Body
	Contacts     []*Manifold

	Debugfn Debugfn
}

// Create a new World abstraction with default params.
func NewWorld() *World {
	w := &World{
		CollisionIterations:   100,
		PenetrationAllowance:  0.05,
		PenetrationCorrection: 0.4,

		BaseTime:    0.0,
		StartTime:   0.0,
		DeltaTime:   1.0 / 60.0 / 10.0 * 1000,
		CurrentTime: 0.0,
		Frequency:   0,

		Accumulator:  0.0,
		StepsCount:   0,
		GravityForce: XY{0.0, 9.81},
		Bodies:       make([]*Body, 0),
		Contacts:     make([]*Manifold, 0),

		Debugfn: nil,
	}

	w.initTimer()
	w.Debug("[PHYSAC] physics module initialized successfully\n")
	w.Accumulator = 0.0

	return w
}

// Run physics step, to be used if PHYSICS_NO_THREADS is set in your main loop.
func (w *World) RunStep() {
	panic("stub")
}

// Sets physics fixed time step in milliseconds. 1.666666 by default.
func (w *World) SetTimeStep(delta float64) {
	panic("stub")
}

// Returns the current amount of created physics bodies.
func (w *World) GetBodiesCount() int {
	panic("stub")
}

// Returns a physics body of the bodies pool at a specific index.
func (w *World) GetBody(index int) *Body {
	panic("stub")
}

// Returns the physics body shape type (PHYSICS_CIRCLE or PHYSICS_POLYGON).
func (w *World) GetShapeType(index int) ShapeType {
	panic("stub")
}

// Returns the amount of vertices of a physics body shape.
func (w *World) GetShapeVerticesCount(index int) int {
	panic("stub")
}

// Unitializes physics pointers and closes physics loop thread.
func (w *World) Close() {
	panic("stub")
}

// Initializes hi-resolution MONOTONIC timer.
func (w *World) initTimer() {
	panic("stub")
}

// Get hi-res MONOTONIC time measure in mseconds.
func (w *World) getTimeCount() uint64 {
	panic("stub")
}

// Get current time measure in milliseconds.
func (w *World) getCurrentTime() float64 {
	panic("stub")
}

// Physics steps calculations (dynamics, collisions and position corrections).
func physicsStep() {
	panic("stub")
}

func (w *World) Debug(f string, as ...interface{}) {
	if w == nil || w.Debugfn == nil {
		return
	}

	w.Debugfn(f, as...)
}
