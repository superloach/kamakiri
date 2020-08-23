package physac

import (
	"math"
	"time"
)

const Epsilon float64 = 0.000001
const K float64 = 1.0 / 3
const Deg2Rad = math.Pi / 180.0

// Abstraction of globals from original Physac lib.
type World struct {
	CollisionIterations   int
	PenetrationAllowance  float64
	PenetrationCorrection float64

	DeltaTime   time.Duration
	CurrentTime time.Duration
	Accumulator time.Duration

	StepsCount   uint
	GravityForce XY
	Bodies       []*Body
	Contacts     []*Contact
}

// Create a new World abstraction with default params.
func NewWorld() *World {
	w := &World{
		CollisionIterations:   100,
		PenetrationAllowance:  0.05,
		PenetrationCorrection: 0.4,

		CurrentTime: 0,
		DeltaTime:   time.Second / 600,
		Accumulator: 0,

		StepsCount:   0,
		GravityForce: XY{0.0, -9.81},
		Bodies:       make([]*Body, 0),
		Contacts:     make([]*Contact, 0),
	}

	return w
}

func (w *World) Delta() float64 {
	return float64(w.DeltaTime) / float64(time.Millisecond)
}

// RunStep is a wrapper to ensure PhysicsStep is run at a fixed time step.
func (w *World) RunStep(delta time.Duration) {
	// Store the time elapsed since the last frame began
	w.Accumulator += delta

	// Fixed time stepping loop
	for w.Accumulator >= w.DeltaTime {
		w.PhysicsStep()
		w.Accumulator -= w.DeltaTime
	}
}

// PhysicsStep performs physics steps calculations (dynamics, collisions and position corrections) for one DeltaTime.
func (w *World) PhysicsStep() {
	// Update current steps count
	w.StepsCount++

	// Clear previous generated collisions information
	for i := len(w.Contacts) - 1; i >= 0; i-- {
		if w.Contacts[i] != nil {
			w.Contacts[i] = nil
		}
	}

	// Reset physics bodies grounded state
	for i := 0; i < len(w.Bodies); i++ {
		w.Bodies[i].IsGrounded = false
	}

	// Generate new collision information
	for i := 0; i < len(w.Bodies); i++ {
		bodyA := w.Bodies[i]

		if bodyA != nil {
			for j := i + 1; j < len(w.Bodies); j++ {
				bodyB := w.Bodies[j]

				if bodyB != nil {
					if bodyA.InverseMass() == 0 && bodyB.InverseMass() == 0 {
						continue
					}

					contact := w.NewContact(bodyA, bodyB)
					contact.solve()

					if contact.Count > 0 {
						// Create a new contact with same information as previously solved contact and add it to the contacts pool last slot
						contact2 := w.NewContact(bodyA, bodyB)
						contact2.Penetration = contact.Penetration
						contact2.Normal = contact.Normal
						contact2.Contacts[0] = contact.Contacts[0]
						contact2.Contacts[1] = contact.Contacts[1]
						contact2.Count = contact.Count
						contact2.Restitution = contact.Restitution
						contact2.DynamicFriction = contact.DynamicFriction
						contact2.StaticFriction = contact.StaticFriction
					}
				}
			}
		}
	}

	// Integrate forces to physics bodies
	for i := 0; i < len(w.Bodies); i++ {
		body := w.Bodies[i]
		if body != nil {
			body.integrateForces()
		}
	}

	// Initialize physics contacts to solve collisions
	for i := 0; i < len(w.Contacts); i++ {
		contact := w.Contacts[i]
		if contact != nil {
			contact.initialize()
		}
	}

	// Integrate physics collisions impulses to solve collisions
	for i := 0; i < w.CollisionIterations; i++ {
		for j := 0; j < len(w.Contacts); j++ {
			contact := w.Contacts[i]
			if contact != nil {
				contact.integrateImpulses()
			}
		}
	}

	// Integrate velocity to physics bodies
	for i := 0; i < len(w.Bodies); i++ {
		body := w.Bodies[i]
		if body != nil {
			body.integrateVelocity()
		}
	}

	// Correct physics bodies positions based on contacts collision information
	for i := 0; i < len(w.Contacts); i++ {
		contact := w.Contacts[i]
		if contact != nil {
			contact.correctPositions()
		}
	}

	// Clear physics bodies forces
	for i := 0; i < len(w.Bodies); i++ {
		body := w.Bodies[i]
		if body != nil {
			body.Force = XY{0, 0}
			body.Torque = 0.0
		}
	}
}
