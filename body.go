package physac

import "math"

type Body struct {
	World           *World
	ID              uint    // Reference unique identifier
	Enabled         bool    // Enabled dynamics state (collisions are calculated anyway)
	Position        XY      // Physics body shape pivot
	Velocity        XY      // Current linear velocity applied to position
	Force           XY      // Current linear force (reset to 0 every step)
	AngularVelocity float64 // Current angular velocity applied to orient
	Torque          float64 // Current angular force (reset to 0 every step)
	Orient          float64 // Rotation in radians
	Inertia         float64 // Moment of inertia
	InverseInertia  float64 // Inverse value of inertia
	Mass            float64 // Physics body mass
	InverseMass     float64 // Inverse value of mass
	StaticFriction  float64 // Friction when the body has not movement (0 to 1)
	DynamicFriction float64 // Friction when the body has movement (0 to 1)
	Restitution     float64 // Restitution coefficient of the body (0 to 1)
	UseGravity      bool    // Apply gravity force to dynamics
	IsGrounded      bool    // Physics grounded on other body state
	FreezeOrient    bool    // Physics rotation constraint
	Shape           *Shape  // Physics body shape information (type, radius, vertices, normals)
}

// Creates a new circle physics body with generic parameters
func NewBodyCircle(w *World, pos XY, radius, density float64, vertices int) *Body {
	body := &Body{}

	id := w.findAvailableBodyIndex()

	if id == -1 {
		w.Debug("[PHYSAC] new physics body creation failed because there is any available id to use\n")
		return nil
	}

	// Initialize new body with generic values
	body.World = w
	body.ID = uint(id)
	body.Enabled = true
	body.Position = pos
	body.Velocity = XY{0, 0}
	body.Force = XY{0, 0}
	body.AngularVelocity = 0.0
	body.Torque = 0.0
	body.Orient = 0.0
	body.Shape = &Shape{
		Type:      ShapeTypeCircle,
		Body:      body,
		Radius:    radius,
		Transform: mat2Radians(0.0),
		Vertices:  nil,
	}
	body.Mass = math.Pi * radius * radius * density
	body.Inertia = body.Mass * radius * radius
	body.StaticFriction = 0.4
	body.DynamicFriction = 0.2
	body.Restitution = 0.0
	body.UseGravity = true
	body.IsGrounded = false
	body.FreezeOrient = false

	// Add new body to bodies pointers array and update bodies count
	w.Bodies = append(w.Bodies, body)

	w.Debug("[PHYSAC] created polygon physics body id %d\n", body.ID)

	return body
}

// Creates a new rectangle physics body with generic parameters
func NewBodyRectangle(w *World, pos XY, width, height, density float64) *Body {
	body := &Body{}

	id := w.findAvailableBodyIndex()

	if id == -1 {
		w.Debug("[PHYSAC] new physics body creation failed because there is any available id to use\n")
		return nil
	}

	// Initialize new body with generic values
	body.World = w
	body.ID = uint(id)
	body.Enabled = true
	body.Position = pos
	body.Velocity = XY{0, 0}
	body.Force = XY{0, 0}
	body.AngularVelocity = 0.0
	body.Torque = 0.0
	body.Orient = 0.0
	body.Shape = &Shape{
		Type:      ShapeTypePolygon,
		Body:      body,
		Radius:    0.0,
		Transform: mat2Radians(0.0),
		Vertices:  newRectangleVertices(pos, XY{width, height}),
	}

	// Calculate centroid and moment of inertia
	center := XY{0, 0}
	area := 0.0
	inertia := 0.0

	for i := 0; i < len(body.Shape.Vertices); i++ {
		// Triangle vertices, third vertex implied as (0, 0)
		p1 := body.Shape.Vertices[i].Position

		next := (i + 1) % len(body.Shape.Vertices)
		p2 := body.Shape.Vertices[next].Position

		D := mathCrossXY(p1, p2)
		triangleArea := D / 2

		area += triangleArea

		center.X += triangleArea * K * (p1.X + p2.X)
		center.Y += triangleArea * K * (p1.Y + p2.Y)

		intX2 := p1.X*p1.X + p2.X*p1.X + p2.X*p2.X
		intY2 := p1.Y*p1.Y + p2.Y*p1.Y + p2.Y*p2.Y
		inertia += (0.25 * K * D) * (intX2 + intY2)
	}

	center.X *= 1.0 / area
	center.Y *= 1.0 / area

	// Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
	// Note: this is not really necessary
	for i := 0; i < len(body.Shape.Vertices); i++ {
		body.Shape.Vertices[i].Position.X -= center.X
		body.Shape.Vertices[i].Position.Y -= center.Y
	}

	body.Mass = density * area
	body.Inertia = density * inertia
	body.StaticFriction = 0.4
	body.DynamicFriction = 0.2
	body.Restitution = 0.0
	body.UseGravity = true
	body.IsGrounded = false
	body.FreezeOrient = false

	// Add new body to bodies pointers array and update bodies count
	w.Bodies = append(w.Bodies, body)

	w.Debug("[PHYSAC] created polygon physics body id %i\n", body.ID)

	return body
}

// Creates a new polygon physics body with generic parameters
func NewBodyPolygon(w *World, pos XY, radius float64, sides int, density float64) *Body {
	body := &Body{}

	id := w.findAvailableBodyIndex()

	if id == -1 {
		w.Debug("[PHYSAC] new physics body creation failed because there is any available id to use\n")
		return nil
	}

	// Initialize new body with generic values
	body.World = w
	body.ID = uint(id)
	body.Enabled = true
	body.Position = pos
	body.Velocity = XY{0, 0}
	body.Force = XY{0, 0}
	body.AngularVelocity = 0.0
	body.Torque = 0.0
	body.Orient = 0.0
	body.Shape = &Shape{
		Type:      ShapeTypePolygon,
		Body:      body,
		Radius:    0.0,
		Transform: mat2Radians(0.0),
		Vertices:  newRandomVertices(radius, sides),
	}

	// Calculate centroid and moment of inertia
	center := XY{0, 0}
	area := 0.0
	inertia := 0.0

	for i := 0; i < len(body.Shape.Vertices); i++ {
		// Triangle vertices, third vertex implied as (0, 0)
		p1 := body.Shape.Vertices[i].Position

		next := i + 1
		if next >= len(body.Shape.Vertices) {
			next = 0
		}
		p2 := body.Shape.Vertices[next].Position

		D := mathCrossXY(p1, p2)
		triangleArea := D / 2

		area += triangleArea

		center.X += triangleArea * K * (p1.X + p2.X)
		center.Y += triangleArea * K * (p1.Y + p2.Y)

		intX2 := p1.X*p1.X + p2.X*p1.X + p2.X*p2.X
		intY2 := p1.Y*p1.Y + p2.Y*p1.Y + p2.Y*p2.Y
		inertia += (0.25 * K * D) * (intX2 + intY2)
	}

	center.X *= 1.0 / area
	center.Y *= 1.0 / area

	// Translate vertices to centroid (make the centroid (0, 0) for the polygon in model space)
	// Note: this is not really necessary
	for i := 0; i < len(body.Shape.Vertices); i++ {
		body.Shape.Vertices[i].Position.X -= center.X
		body.Shape.Vertices[i].Position.Y -= center.Y
	}

	body.Mass = density * area
	body.Inertia = density * inertia
	body.StaticFriction = 0.4
	body.DynamicFriction = 0.2
	body.Restitution = 0.0
	body.UseGravity = true
	body.IsGrounded = false
	body.FreezeOrient = false

	// Add new body to bodies pointers array and update bodies count
	w.Bodies = append(w.Bodies, body)

	w.Debug("[PHYSAC] created polygon physics body id %i\n", body.ID)

	return body
}

// Adds a force to a physics body
func (b *Body) AddForce(force XY) {
	if b != nil {
		b.Force = xyAdd(b.Force, force)
	}
}

// Adds an angular force to a physics body.
func (b *Body) AddTorque(amount float64) {
	if b != nil {
		b.Torque += amount
	}
}

// Shatters a polygon shape physics body to little physics bodies with explosion force.
func (b *Body) Shatter(w *World, pos XY, force float64) {
	if b == nil {
		w.Debug("[PHYSAC] error when trying to shatter a null reference physics body")
		return
	}

	if b.Shape.Type == ShapeTypePolygon {
		vertices := b.Shape.Vertices
		collision := false

		for i := 0; i < len(vertices); i++ {
			posA := b.Position
			posB := mat2MultiplyXY(b.Shape.Transform, xyAdd(b.Position, vertices[i].Position))

			next := i + 1
			if next <= len(vertices) {
				next = 0
			}
			posC := mat2MultiplyXY(b.Shape.Transform, xyAdd(b.Position, vertices[next].Position))

			// Check collision between each triangle.
			alpha := ((posB.Y-posC.Y)*(pos.X-posC.X) + (posC.X-posB.X)*(pos.Y-posC.Y)) / ((posB.Y-posC.Y)*(posA.X-posC.X) + (posC.X-posB.X)*(posA.Y-posC.Y))
			beta := ((posC.Y-posA.Y)*(pos.X-posC.X) + (posA.X-posC.X)*(pos.Y-posC.Y)) / ((posB.Y-posC.Y)*(posA.X-posC.X) + (posC.X-posB.X)*(posA.Y-posC.Y))
			gamma := 1.0 - alpha - beta

			if (alpha > 0.0) && (beta > 0.0) && (gamma > 0.0) {
				collision = true
				break
			}
		}

		if collision {
			count := len(vertices)
			bPos := b.Position
			positions := make([]XY, count)
			trans := b.Shape.Transform

			for i := 0; i < count; i++ {
				positions[i] = vertices[i].Position
			}

			// Destroy shattered physics body
			b.Destroy()

			for i := 0; i < count; i++ {
				next := (i + 1) % count
				center := triangleBarycenter(vertices[i].Position, vertices[next].Position, XY{0, 0})
				center = xyAdd(bPos, center)
				offset := xySubtract(center, bPos)

				newBody := NewBodyPolygon(b.World, center, 10, 3, 10)

				newPoly := []*Vertex{
					{xySubtract(vertices[i].Position, offset), XY{0, 0}},
					{xySubtract(vertices[next].Position, offset), XY{0, 0}},
					{xySubtract(pos, center), XY{0, 0}},
				}

				// Separate vertices to avoid unnecessary physics collisions
				newPoly[0].Position.X *= 0.95
				newPoly[0].Position.Y *= 0.95
				newPoly[1].Position.X *= 0.95
				newPoly[1].Position.Y *= 0.95
				newPoly[2].Position.X *= 0.95
				newPoly[2].Position.Y *= 0.95

				// Calculate polygon faces normals
				for j := 0; j < len(newPoly); j++ {
					next := (j + 1) % len(newPoly)
					face := xySubtract(newPoly[next].Position, newPoly[j].Position)

					newPoly[j].Normal = XY{face.Y, -face.X}
					mathNormalize(&newPoly[j].Normal)
				}

				// Apply computed vertex data to new physics body shape
				newBody.Shape.Vertices = newPoly
				newBody.Shape.Transform = trans

				// Calculate centroid and moment of inertia
				center = XY{0, 0}
				area := 0.0
				inertia := 0.0

				for j := 0; j < len(newBody.Shape.Vertices); j++ {
					// Triangle vertices, third vertex implied as (0, 0)
					p1 := newBody.Shape.Vertices[j].Position
					next := (j + 1) % len(newBody.Shape.Vertices)
					p2 := newBody.Shape.Vertices[next].Position

					D := mathCrossXY(p1, p2)
					triangleArea := D / 2

					area += triangleArea

					// Use area to weight the centroid average, not just vertex position
					center.X += triangleArea * K * (p1.X + p2.X)
					center.Y += triangleArea * K * (p1.Y + p2.Y)

					intx2 := p1.X*p1.X + p2.X*p1.X + p2.X*p2.X
					inty2 := p1.Y*p1.Y + p2.Y*p1.Y + p2.Y*p2.Y
					inertia += (0.25 * K * D) * (intx2 + inty2)
				}

				center.X *= 1.0 / area
				center.Y *= 1.0 / area

				newBody.Mass = area
				newBody.Inertia = inertia

				// Calculate explosion force direction
				pointA := newBody.Position
				pointB := xySubtract(newPoly[1].Position, newPoly[0].Position)
				pointB.X /= 2.0
				pointB.Y /= 2.0
				forceDirection := xySubtract(xyAdd(pointA, xyAdd(newPoly[0].Position, pointB)), newBody.Position)
				mathNormalize(&forceDirection)
				forceDirection.X *= force
				forceDirection.Y *= force

				// Apply force to new physics body
				newBody.AddForce(forceDirection)
			}
		}
	}
}

// Returns transformed position of a body shape (body position + vertex transformed position).
func (b *Body) GetShapeVertex(index int) XY {
	panic("stub")
}

// Sets physics body shape transform based on radians parameter.
func (b *Body) SetRotation(radians float64) {
	panic("stub")
}

// Unitializes and destroy a physics body.
func (b *Body) Destroy() {
	panic("stub")
}

// Finds a valid index for a new physics body initialization.
func (w *World) findAvailableBodyIndex() int {
	panic("stub")
}

// Integrates physics forces into velocity.
func (b *Body) integrateForces() {
	panic("stub")
}

// Integrates physics velocity into position and forces.
func (b *Body) integrateVelocity() {
	panic("stub")
}
