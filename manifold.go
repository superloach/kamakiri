package physac

type Manifold struct {
	ID              uint    // Reference unique identifier.
	BodyA           *Body   // Manifold first physics body reference.
	BodyB           *Body   // Manifold second physics body reference.
	Penetration     float64 // Depth of penetration from collision.
	Normal          XY      // Normal direction vector from 'a' to 'b'.
	Contacts        [2]XY   // Points of contact during collision.
	ContactsCount   uint    // Current collision number of contacts.
	Restitution     float64 // Mixed restitution during collision.
	DynamicFriction float64 // Mixed dynamic friction during collision.
	StaticFriction  float64 // Mixed static friction during collision.
}

// Finds a valid index for a new manifold initialization.
func findAvailableManifoldIndex() int {
	panic("stub")
}

// Creates a new physics manifold to solve collision.
func newManifold(a, b *Body) *Manifold {
	panic("stub")
}

// Unitializes and destroys a physics manifold.
func (m *Manifold) destroy() {
	panic("stub")
}

// Solves a created physics manifold between two physics bodies.
func (m *Manifold) solve() {
	panic("stub")
}

// Solves collision between two circle shape physics bodies.
func (m *Manifold) solveCircleToCircle() {
	panic("stub")
}

// Solves collision between a circle to a polygon shape physics bodies.
func (m *Manifold) solveCircleToPolygon() {
	panic("stub")
}

// Solves collision between a polygon to a circle shape physics bodies.
func (m *Manifold) solvePolygonToCircle() {
	panic("stub")
}

// Solve collision between two different types of shapes.
func (m *Manifold) solveDifferentShapes(a, b *Body) {
	panic("stub")
}

// Solves collision between two polygons shape physics bodies.
func (m *Manifold) solvePolygonToPolygon() {
	panic("stub")
}

// Initializes physics manifolds to solve collisions.
func (m *Manifold) initialize() {
	panic("stub")
}

// Integrates physics collisions impulses to solve collisions.
func (m *Manifold) integrateImpulses() {
	panic("stub")
}

// Corrects physics bodies positions based on manifolds collision information.
func (m *Manifold) correctPositions() {
	panic("stub")
}
