package physac

import (
	"math"
)

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
	Mass            float64 // Physics body mass
	StaticFriction  float64 // Friction when the body has not movement (0 to 1)
	DynamicFriction float64 // Friction when the body has movement (0 to 1)
	Restitution     float64 // Restitution coefficient of the body (0 to 1)
	UseGravity      bool    // Apply gravity force to dynamics
	IsGrounded      bool    // Physics grounded on other body state
	FreezeOrient    bool    // Physics rotation constraint
	Shape           *Shape  // Physics body shape information (type, radius, vertices, normals)
}

// Finds a valid index for a new physics body initialization.
func (w *World) findAvailableBodyIndex() uint {
	if len(w.Bodies) == 0 {
		return 0
	}

	for i := uint(0); ; i++ {
		seen := false

		for _, body := range w.Bodies {
			if body.ID == i {
				seen = true
				break
			}
		}

		if !seen {
			return i
		}
	}
}

// Inverse value of inertia
func (b *Body) InverseInertia() float64 {
	if b.Inertia == 0.0 {
		return 0.0
	}
	return 1 / b.Inertia
}

// Inverse value of mass
func (b *Body) InverseMass() float64 {
	if b.Mass == 0.0 {
		return 0.0
	}
	return 1 / b.Mass
}

// Creates a new circle physics body with generic parameters
func (w *World) NewBodyCircle(pos XY, radius, density float64, vertices int) *Body {
	// Initialize new body with generic values
	body := &Body{
		World:           w,
		ID:              w.findAvailableBodyIndex(),
		Enabled:         true,
		Position:        pos,
		Velocity:        XY{0, 0},
		Force:           XY{0, 0},
		AngularVelocity: 0.0,
		Torque:          0.0,
		Orient:          0.0,
		Mass:            math.Pi * radius * radius * density,
		StaticFriction:  0.4,
		DynamicFriction: 0.2,
		Restitution:     0.0,
		UseGravity:      true,
		IsGrounded:      false,
		FreezeOrient:    false,
	}
	body.Inertia = body.Mass * radius * radius
	body.Shape = &Shape{
		Type:      ShapeTypeCircle,
		Body:      body,
		Radius:    radius,
		Transform: Mat2{},
		Vertices:  make([]Vertex, vertices),
	}

	// Add new body to bodies pointers array and update bodies count
	w.Bodies = append(w.Bodies, body)

	return body
}

// Creates a new rectangle physics body with generic parameters
func (w *World) NewBodyRectangle(pos XY, width, height, density float64) *Body {
	body := &Body{}

	// Initialize new body with generic values
	body.World = w
	body.ID = w.findAvailableBodyIndex()
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
		Transform: Mat2{},
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

		D := p1.CrossXY(p2)
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

	return body
}

// Creates a new polygon physics body with generic parameters
func (w *World) NewBodyPolygon(pos XY, radius float64, sides int, density float64) *Body {
	body := &Body{}

	// Initialize new body with generic values
	body.World = w
	body.ID = w.findAvailableBodyIndex()
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
		Transform: Mat2{},
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

		D := p1.CrossXY(p2)
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

	return body
}

// Adds a force to a physics body
func (b *Body) AddForce(force XY) {
	if b != nil {
		b.Force = b.Force.Add(force)
	}
}

// Adds an angular force to a physics body.
func (b *Body) AddTorque(amount float64) {
	if b != nil {
		b.Torque += amount
	}
}

// Shatters a polygon shape physics body to little physics bodies with explosion force.
func (b *Body) Shatter(pos XY, force float64) {
	if b == nil {
		return
	}

	if b.Shape.Type == ShapeTypePolygon {
		vertices := b.Shape.Vertices
		collision := false

		for i := 0; i < len(vertices); i++ {
			posA := b.Position
			posB := b.Shape.Transform.MultiplyXY(b.Position.Add(vertices[i].Position))

			next := i + 1
			if next <= len(vertices) {
				next = 0
			}
			posC := b.Shape.Transform.MultiplyXY(b.Position.Add(vertices[next].Position))

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
				center := TriangleBarycenter(vertices[i].Position, vertices[next].Position, XY{0, 0})
				center = bPos.Add(center)
				offset := center.Subtract(bPos)

				newBody := b.World.NewBodyPolygon(center, 10, 3, 10)

				newPoly := []Vertex{
					{vertices[i].Position.Subtract(offset), XY{0, 0}},
					{vertices[next].Position.Subtract(offset), XY{0, 0}},
					{pos.Subtract(center), XY{0, 0}},
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
					face := newPoly[next].Position.Subtract(newPoly[j].Position)

					newPoly[j].Normal = XY{face.Y, -face.X}
					newPoly[j].Normal = newPoly[j].Normal.Normalize()
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

					D := p1.CrossXY(p2)
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
				pointB := newPoly[1].Position.Subtract(newPoly[0].Position)
				pointB.X /= 2.0
				pointB.Y /= 2.0
				forceDirection := pointA.Add(newPoly[0].Position.Add(pointB)).Subtract(newBody.Position)
				forceDirection = forceDirection.Normalize()
				forceDirection.X *= force
				forceDirection.Y *= force

				// Apply force to new physics body
				newBody.AddForce(forceDirection)
			}
		}
	}
}

// Returns transformed position of a body shape (body position + vertex transformed position).
func (b *Body) GetShapeVertex(vertex int) XY {
	position := XY{0.0, 0.0}

	if b == nil {
		return position
	}

	switch b.Shape.Type {
	case ShapeTypeCircle:
		position.X = b.Position.X + math.Cos(360.0/float64(len(b.Shape.Vertices)*vertex)*Deg2Rad)*b.Shape.Radius
		position.Y = b.Position.Y + math.Sin(360.0/float64(len(b.Shape.Vertices)*vertex)*Deg2Rad)*b.Shape.Radius
	case ShapeTypePolygon:
		vertexData := b.Shape.Vertices
		position = b.Position.Add(b.Shape.Transform.MultiplyXY(vertexData[vertex].Position))
	default:
	}

	return position
}

// Sets physics body shape transform based on radians parameter.
func (b *Body) SetRotation(radians float64) {
	if b == nil {
		return
	}

	b.Orient = radians

	if b.Shape.Type == ShapeTypePolygon {
		b.Shape.Transform = Mat2Radians(radians)
	}
}

// Unitializes and destroy a physics body.
func (b *Body) Destroy() {
	if b == nil {
		return
	}

	id := b.ID
	index := -1

	for i := 0; i < len(b.World.Bodies); i++ {
		if b.World.Bodies[i].ID == id {
			index = i
			break
		}
	}

	if index == -1 {
		return
	}

	// Free body allocated memory
	b.World.Bodies[index] = b.World.Bodies[len(b.World.Bodies)-1]
	b.World.Bodies[len(b.World.Bodies)-1] = nil
	b.World.Bodies = b.World.Bodies[:len(b.World.Bodies)-1]
}

// Integrates physics forces into velocity.
func (b *Body) integrateForces() {
	imass := b.InverseMass()
	if (b == nil) || (imass == 0.0) || !b.Enabled {
		return
	}

	b.Velocity.X += (b.Force.X * imass) * (b.World.Delta() / 2)
	b.Velocity.Y += (b.Force.Y * imass) * (b.World.Delta() / 2)

	if b.UseGravity {
		b.Velocity.X += b.World.GravityForce.X * (b.World.Delta() / 1000 / 2)
		b.Velocity.Y += b.World.GravityForce.Y * (b.World.Delta() / 1000 / 2)
	}

	if !b.FreezeOrient {
		b.AngularVelocity += b.Torque * b.InverseInertia() * (b.World.Delta() / 2)
	}
}

// Integrates physics velocity into position and forces.
func (b *Body) integrateVelocity() {
	if b == nil || !b.Enabled {
		return
	}

	b.Position.X += b.Velocity.X * b.World.Delta()
	b.Position.Y += b.Velocity.Y * b.World.Delta()

	if !b.FreezeOrient {
		b.Orient += b.AngularVelocity * b.World.Delta()
	}

	b.Shape.Transform = Mat2Radians(b.Orient)

	b.integrateForces()
}
