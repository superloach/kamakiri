package physac

import "math"

type Contact struct {
	World           *World
	ID              uint    // Reference unique identifier.
	BodyA           *Body   // Contact first physics body reference.
	BodyB           *Body   // Contact second physics body reference.
	Penetration     float64 // Depth of penetration from collision.
	Normal          XY      // Normal direction vector from 'a' to 'b'.
	Contacts        [2]XY   // Points of contact during collision.
	Count           uint    // Current collision number of contacts.
	Restitution     float64 // Mixed restitution during collision.
	DynamicFriction float64 // Mixed dynamic friction during collision.
	StaticFriction  float64 // Mixed static friction during collision.
}

// Finds a valid index for a new contact initialization.
func (w *World) findAvailableContactIndex() int {
	if len(w.Contacts) == 0 {
		return 0
	}

	for i := 0; ; i++ {
		for _, contact := range w.Contacts {
			if contact.ID == uint(i) {
				continue
			}

			return i
		}
	}
}

// Creates a new physics contact to solve collision.
func newContact(w *World, a, b *Body) *Contact {
	contact := &Contact{}

	newID := w.findAvailableContactIndex()

	if newID == -1 {
		w.Debug("new contact creation failed because there is any available id to use")
		return nil
	}

	// Initialize new contact with generic values
	contact.ID = uint(newID)
	contact.BodyA = a
	contact.BodyB = b
	contact.Penetration = 0
	contact.Normal = XY{0, 0}
	contact.Contacts[0] = XY{0, 0}
	contact.Contacts[1] = XY{0, 0}
	contact.Count = 0
	contact.Restitution = 0.0
	contact.DynamicFriction = 0.0
	contact.StaticFriction = 0.0

	// Add new body to bodies pointers array and update bodies count
	w.Contacts = append(w.Contacts, contact)

	return contact
}

// Unitializes and destroys a physics contact.
func (c *Contact) destroy() {
	if c == nil {
		panic("tried to destroy a nil contact")
	}

	id := c.ID
	index := -1

	for i := 0; i < len(c.World.Contacts); i++ {
		if c.World.Contacts[i].ID == id {
			index = i
			break
		}
	}

	if index == -1 {
		c.World.Debug("Not possible to contact id %i in pointers array", id)
		return
	}

	// Free contact allocated memory
	c.World.Contacts[index] = c.World.Contacts[len(c.World.Contacts)-1]
	c.World.Contacts[len(c.World.Contacts)-1] = nil
	c.World.Contacts = c.World.Contacts[:len(c.World.Contacts)-1]
}

// Solves a created physics contact between two physics bodies.
func (c *Contact) solve() {
	switch c.BodyA.Shape.Type {
	case ShapeTypeCircle:
		switch c.BodyB.Shape.Type {
		case ShapeTypeCircle:
			c.solveCircleToCircle()
		case ShapeTypePolygon:
			c.solveCircleToPolygon()
		default:
		}
	case ShapeTypePolygon:
		switch c.BodyB.Shape.Type {
		case ShapeTypeCircle:
			c.solvePolygonToCircle()
		case ShapeTypePolygon:
			c.solvePolygonToPolygon()
		default:
		}
	default:
	}

	// Update physics body grounded state if normal direction is down and grounded state is not set yet in previous contacts
	if !c.BodyB.IsGrounded {
		c.BodyB.IsGrounded = (c.Normal.Y < 0)
	}
}

// Solves collision between two circle shape physics bodies.
func (c *Contact) solveCircleToCircle() {
	bodyA := c.BodyA
	bodyB := c.BodyB

	if bodyA == nil || bodyB == nil {
		return
	}

	// Calculate translational vector, which is normal
	normal := xySubtract(bodyB.Position, bodyA.Position)

	distSqr := mathLenSqr(normal)
	radius := bodyA.Shape.Radius + bodyB.Shape.Radius

	// Check if circles are not in contact
	if distSqr >= radius*radius {
		c.Count = 0
		return
	}

	distance := math.Sqrt(distSqr)
	c.Count = 1

	if distance == 0.0 {
		c.Penetration = bodyA.Shape.Radius
		c.Normal = XY{1.0, 0.0}
		c.Contacts[0] = bodyA.Position
	} else {
		c.Penetration = radius - distance
		c.Normal = XY{normal.X / distance, normal.Y / distance} // Faster than using mathNormalize() due to sqrt is already performed
		c.Contacts[0] = XY{c.Normal.X*bodyA.Shape.Radius + bodyA.Position.X, c.Normal.Y*bodyA.Shape.Radius + bodyA.Position.Y}
	}

	// Update physics body grounded state if normal direction is down
	if !bodyA.IsGrounded {
		bodyA.IsGrounded = (c.Normal.Y < 0)
	}
}

// Solves collision between a circle to a polygon shape physics bodies.
func (c *Contact) solveCircleToPolygon() {
	bodyA := c.BodyA
	bodyB := c.BodyB

	if bodyA == nil || bodyB == nil {
		return
	}

	c.solveDifferentShapes(bodyA, bodyB)
}

// Solves collision between a polygon to a circle shape physics bodies.
func (c *Contact) solvePolygonToCircle() {
	bodyA := c.BodyA
	bodyB := c.BodyB

	if (bodyA == nil) || (bodyB == nil) {
		return
	}

	c.solveDifferentShapes(bodyB, bodyA)

	c.Normal.X *= -1.0
	c.Normal.Y *= -1.0
}

// Solve collision between two different types of shapes.
func (c *Contact) solveDifferentShapes(a, b *Body) {
	c.Count = 0

	// Transform circle center to polygon transform space
	center := a.Position
	center = mat2MultiplyXY(mat2Transpose(b.Shape.Transform), xySubtract(center, b.Position))

	// Find edge with minimum penetration
	// It is the same concept as using support points in SolvePolygonToPolygon
	separation := -math.MaxFloat64
	faceNormal := 0
	vertexData := b.Shape.Vertices

	for i := 0; i < len(vertexData); i++ {
		currentSeparation := mathDot(vertexData[i].Normal, xySubtract(center, vertexData[i].Position))

		if currentSeparation > a.Shape.Radius {
			return
		}

		if currentSeparation > separation {
			separation = currentSeparation
			faceNormal = i
		}
	}

	// Grab face's vertices
	v1 := vertexData[faceNormal].Position
	next := (faceNormal + 1) % len(vertexData)
	v2 := vertexData[next].Position

	// Check to see if center is within polygon
	if separation < Epsilon {
		c.Count = 1
		normal := mat2MultiplyXY(b.Shape.Transform, vertexData[faceNormal].Normal)
		c.Normal = XY{-normal.X, -normal.Y}
		c.Contacts[0] = XY{c.Normal.X*a.Shape.Radius + a.Position.X, c.Normal.Y*a.Shape.Radius + a.Position.Y}
		c.Penetration = a.Shape.Radius
		return
	}

	// Determine which voronoi region of the edge center of circle lies within
	dot1 := mathDot(xySubtract(center, v1), xySubtract(v2, v1))
	dot2 := mathDot(xySubtract(center, v2), xySubtract(v1, v2))
	c.Penetration = a.Shape.Radius - separation

	if dot1 <= 0.0 { // Closest to v1
		if distSqr(center, v1) > a.Shape.Radius*a.Shape.Radius {
			return
		}

		c.Count = 1
		normal := xySubtract(v1, center)
		normal = mat2MultiplyXY(b.Shape.Transform, normal)
		mathNormalize(&normal)
		c.Normal = normal
		v1 = mat2MultiplyXY(b.Shape.Transform, v1)
		v1 = xyAdd(v1, b.Position)
		c.Contacts[0] = v1
	} else if dot2 <= 0.0 { // Closest to v2
		if distSqr(center, v2) > a.Shape.Radius*a.Shape.Radius {
			return
		}

		c.Count = 1
		normal := xySubtract(v2, center)
		v2 = mat2MultiplyXY(b.Shape.Transform, v2)
		v2 = xyAdd(v2, b.Position)
		c.Contacts[0] = v2
		normal = mat2MultiplyXY(b.Shape.Transform, normal)
		mathNormalize(&normal)
		c.Normal = normal
	} else { // Closest to face
		normal := vertexData[faceNormal].Normal

		if mathDot(xySubtract(center, v1), normal) > a.Shape.Radius {
			return
		}

		normal = mat2MultiplyXY(b.Shape.Transform, normal)
		c.Normal = XY{-normal.X, -normal.Y}
		c.Contacts[0] = XY{c.Normal.X*a.Shape.Radius + a.Position.X, c.Normal.Y*a.Shape.Radius + a.Position.Y}
		c.Count = 1
	}
}

// Solves collision between two polygons shape physics bodies.
func (c *Contact) solvePolygonToPolygon() {
	if (c.BodyA == nil) || (c.BodyB == nil) {
		return
	}

	bodyA := c.BodyA.Shape
	bodyB := c.BodyB.Shape
	c.Count = 0

	// Check for separating axis with A shape's face planes
	faceA, penetrationA := findAxisLeastPenetration(bodyA, bodyB)
	if penetrationA >= 0.0 {
		return
	}

	// Check for separating axis with B shape's face planes
	faceB, penetrationB := findAxisLeastPenetration(bodyB, bodyA)
	if penetrationB >= 0.0 {
		return
	}

	referenceIndex := 0
	flip := false // Always point from A shape to B shape

	var refPoly *Shape // Reference
	var incPoly *Shape // Incident

	// Determine which shape contains reference face
	if biasGreaterThan(penetrationA, penetrationB) {
		refPoly = bodyA
		incPoly = bodyB
		referenceIndex = faceA
	} else {
		refPoly = bodyB
		incPoly = bodyA
		referenceIndex = faceB
		flip = true
	}

	// World space incident face
	var incidentFace [2]XY
	findIncidentFace(&incidentFace[0], &incidentFace[1], refPoly, incPoly, referenceIndex)

	// Setup reference face vertices
	refData := refPoly.Vertices
	v1 := refData[referenceIndex].Position
	referenceIndex = (referenceIndex + 1) % len(refData)
	v2 := refData[referenceIndex].Position

	// Transform vertices to world space
	v1 = mat2MultiplyXY(refPoly.Transform, v1)
	v1 = xyAdd(v1, refPoly.Body.Position)
	v2 = mat2MultiplyXY(refPoly.Transform, v2)
	v2 = xyAdd(v2, refPoly.Body.Position)

	// Calculate reference face side normal in world space
	sidePlaneNormal := xySubtract(v2, v1)
	mathNormalize(&sidePlaneNormal)

	// Orthogonalize
	refFaceNormal := XY{sidePlaneNormal.Y, -sidePlaneNormal.X}
	refC := mathDot(refFaceNormal, v1)
	negSide := mathDot(sidePlaneNormal, v1) * -1
	posSide := mathDot(sidePlaneNormal, v2)

	// clip incident face to reference face side planes (due to floating point error, possible to not have required points
	if (clip(XY{-sidePlaneNormal.X, -sidePlaneNormal.Y}, negSide, &incidentFace[0], &incidentFace[1]) < 2) {
		return
	}

	if clip(sidePlaneNormal, posSide, &incidentFace[0], &incidentFace[1]) < 2 {
		return
	}

	// Flip normal if required
	c.Normal = refFaceNormal
	if flip {
		c.Normal.X *= -1
		c.Normal.Y *= -1
	}

	// Keep points behind reference face
	currentPoint := 0 // clipped points behind reference face
	separation := mathDot(refFaceNormal, incidentFace[0]) - refC

	if separation <= 0.0 {
		c.Contacts[currentPoint] = incidentFace[0]
		c.Penetration = -separation
		currentPoint++
	} else {
		c.Penetration = 0.0
	}

	separation = mathDot(refFaceNormal, incidentFace[1]) - refC

	if separation <= 0.0 {
		c.Contacts[currentPoint] = incidentFace[1]
		c.Penetration += -separation
		currentPoint++

		// Calculate total penetration average
		c.Penetration /= float64(currentPoint)
	}

	c.Count = uint(currentPoint)
}

// Initializes physics contacts to solve collisions.
func (c *Contact) initialize() {
	bodyA := c.BodyA
	bodyB := c.BodyB

	if (bodyA == nil) || (bodyB == nil) {
		return
	}

	// Calculate average restitution, static and dynamic friction
	c.Restitution = math.Sqrt(bodyA.Restitution * bodyB.Restitution)
	c.StaticFriction = math.Sqrt(bodyA.StaticFriction * bodyB.StaticFriction)
	c.DynamicFriction = math.Sqrt(bodyA.DynamicFriction * bodyB.DynamicFriction)

	for i := 0; i < int(c.Count); i++ {
		// Caculate radius from center of mass to contact
		radiusA := xySubtract(c.Contacts[i], bodyA.Position)
		radiusB := xySubtract(c.Contacts[i], bodyB.Position)

		crossA := mathCross(bodyA.AngularVelocity, radiusA)
		crossB := mathCross(bodyB.AngularVelocity, radiusB)

		radiusV := XY{0.0, 0.0}
		radiusV.X = bodyB.Velocity.X + crossB.X - bodyA.Velocity.X - crossA.X
		radiusV.Y = bodyB.Velocity.Y + crossB.Y - bodyA.Velocity.Y - crossA.Y

		// Determine if we should perform a resting collision or not;
		// The idea is if the only thing moving this object is gravity, then the collision should be performed without any restitution
		if mathLenSqr(radiusV) < mathLenSqr(XY{
			c.World.GravityForce.X * c.World.DeltaTime / 1000,
			c.World.GravityForce.Y * c.World.DeltaTime / 1000,
		})+Epsilon {
			c.Restitution = 0
		}
	}
}

// Integrates physics collisions impulses to solve collisions.
func (c *Contact) integrateImpulses() {
	bodyA := c.BodyA
	bodyB := c.BodyB

	if (bodyA == nil) || (bodyB == nil) {
		return
	}

	// Early out and positional correct if both objects have infinite mass
	if math.Abs(bodyA.InverseMass()+bodyB.InverseMass()) <= Epsilon {
		bodyA.Velocity = XY{0, 0}
		bodyB.Velocity = XY{0, 0}
		return
	}

	for i := 0; i < int(c.Count); i++ {
		// Calculate radius from center of mass to contact
		radiusA := xySubtract(c.Contacts[i], bodyA.Position)
		radiusB := xySubtract(c.Contacts[i], bodyB.Position)

		// Calculate relative velocity
		radiusV := XY{0.0, 0.0}
		radiusV.X = bodyB.Velocity.X + mathCross(bodyB.AngularVelocity, radiusB).X - bodyA.Velocity.X - mathCross(bodyA.AngularVelocity, radiusA).X
		radiusV.Y = bodyB.Velocity.Y + mathCross(bodyB.AngularVelocity, radiusB).Y - bodyA.Velocity.Y - mathCross(bodyA.AngularVelocity, radiusA).Y

		// Relative velocity along the normal
		contactVelocity := mathDot(radiusV, c.Normal)

		// Do not resolve if velocities are separating
		if contactVelocity > 0.0 {
			return
		}

		raCrossN := mathCrossXY(radiusA, c.Normal)
		rbCrossN := mathCrossXY(radiusB, c.Normal)

		inverseMassSum := bodyA.InverseMass() + bodyB.InverseMass() + (raCrossN*raCrossN)*bodyA.InverseInertia() + (rbCrossN*rbCrossN)*bodyB.InverseInertia()

		// Calculate impulse scalar value
		impulse := -(1.0 + c.Restitution) * contactVelocity
		impulse /= inverseMassSum
		impulse /= float64(c.Count)

		// Apply impulse to each physics body
		impulseV := XY{c.Normal.X * impulse, c.Normal.Y * impulse}

		if bodyA.Enabled {
			bodyA.Velocity.X += bodyA.InverseMass() * (-impulseV.X)
			bodyA.Velocity.Y += bodyA.InverseMass() * (-impulseV.Y)

			if !bodyA.FreezeOrient {
				bodyA.AngularVelocity += bodyA.InverseInertia() * mathCrossXY(radiusA, XY{-impulseV.X, -impulseV.Y})
			}
		}

		if bodyB.Enabled {
			bodyB.Velocity.X += bodyB.InverseMass() * (impulseV.X)
			bodyB.Velocity.Y += bodyB.InverseMass() * (impulseV.Y)

			if !bodyB.FreezeOrient {
				bodyB.AngularVelocity += bodyB.InverseInertia() * mathCrossXY(radiusB, impulseV)
			}
		}

		// Apply friction impulse to each physics body
		radiusV.X = bodyB.Velocity.X + mathCross(bodyB.AngularVelocity, radiusB).X - bodyA.Velocity.X - mathCross(bodyA.AngularVelocity, radiusA).X
		radiusV.Y = bodyB.Velocity.Y + mathCross(bodyB.AngularVelocity, radiusB).Y - bodyA.Velocity.Y - mathCross(bodyA.AngularVelocity, radiusA).Y

		tangent := XY{radiusV.X - (c.Normal.X * mathDot(radiusV, c.Normal)), radiusV.Y - (c.Normal.Y * mathDot(radiusV, c.Normal))}
		mathNormalize(&tangent)

		// Calculate impulse tangent magnitude
		impulseTangent := -mathDot(radiusV, tangent)
		impulseTangent /= inverseMassSum
		impulseTangent /= float64(c.Count)

		absImpulseTangent := math.Abs(impulseTangent)

		// Don't apply tiny friction impulses
		if absImpulseTangent <= Epsilon {
			return
		}

		// Apply coulumb's law
		tangentImpulse := XY{0.0, 0.0}
		if absImpulseTangent < impulse*c.StaticFriction {
			tangentImpulse = XY{tangent.X * impulseTangent, tangent.Y * impulseTangent}
		} else {
			tangentImpulse = XY{tangent.X * -impulse * c.DynamicFriction, tangent.Y * -impulse * c.DynamicFriction}
		}

		// Apply friction impulse
		if bodyA.Enabled {
			bodyA.Velocity.X += bodyA.InverseMass() * (-tangentImpulse.X)
			bodyA.Velocity.Y += bodyA.InverseMass() * (-tangentImpulse.Y)

			if !bodyA.FreezeOrient {
				bodyA.AngularVelocity += bodyA.InverseInertia() * mathCrossXY(radiusA, XY{-tangentImpulse.X, -tangentImpulse.Y})
			}
		}

		if bodyB.Enabled {
			bodyB.Velocity.X += bodyB.InverseMass() * (tangentImpulse.X)
			bodyB.Velocity.Y += bodyB.InverseMass() * (tangentImpulse.Y)

			if !bodyB.FreezeOrient {
				bodyB.AngularVelocity += bodyB.InverseInertia() * mathCrossXY(radiusB, tangentImpulse)
			}
		}
	}
}

// Corrects physics bodies positions based on contacts collision information.
func (c *Contact) correctPositions() {
	bodyA := c.BodyA
	bodyB := c.BodyB

	if (bodyA == nil) || (bodyB == nil) {
		return
	}

	correction := XY{0.0, 0.0}
	correction.X = (math.Max(c.Penetration-c.World.PenetrationAllowance, 0.0) / (bodyA.InverseMass() + bodyB.InverseMass())) * c.Normal.X * c.World.PenetrationCorrection
	correction.Y = (math.Max(c.Penetration-c.World.PenetrationAllowance, 0.0) / (bodyA.InverseMass() + bodyB.InverseMass())) * c.Normal.Y * c.World.PenetrationCorrection

	if bodyA.Enabled {
		bodyA.Position.X -= correction.X * bodyA.InverseMass()
		bodyA.Position.Y -= correction.Y * bodyA.InverseMass()
	}

	if bodyB.Enabled {
		bodyB.Position.X += correction.X * bodyB.InverseMass()
		bodyB.Position.Y += correction.Y * bodyB.InverseMass()
	}
}
