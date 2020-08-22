package physac

import "math"

type ShapeType uint8

const (
	ShapeTypeCircle ShapeType = iota
	ShapeTypePolygon
)

type Shape struct {
	Type      ShapeType // Physics shape type (circle or polygon).
	Body      *Body     // Shape physics body reference.
	Radius    float64   // Circle shape radius (used for circle shapes).
	Transform Mat2      // Vertices transform matrix 2x2.
	Vertices  []Vertex  // Polygon shape vertices position and normals verts (just used for polygon shapes).
}

// Finds polygon shapes axis least penetration.
func findAxisLeastPenetration(a, b *Shape) (int, float64) {
	bestDistance := -math.MaxFloat64
	bestIndex := 0

	vertsA := a.Vertices

	for i := 0; i < len(vertsA); i++ {
		// Retrieve a face normal from A shape
		normal := vertsA[i].Normal
		transNormal := a.Transform.MultiplyXY(normal)

		// Transform face normal into B shape's model space
		buT := b.Transform.Transpose()
		normal = buT.MultiplyXY(transNormal)

		// Retrieve support point from B shape along -n
		support := b.getSupport(XY{-normal.X, -normal.Y})

		// Retrieve vertex on face from A shape, transform into B shape's model space
		vertex := vertsA[i].Position
		vertex = a.Transform.MultiplyXY(vertex)
		vertex = vertex.Subtract(a.Body.Position)
		vertex = vertex.Subtract(b.Body.Position)
		vertex = buT.MultiplyXY(vertex)

		// Compute penetration distance in B shape's model space
		distance := normal.Dot(support.Subtract(vertex))

		// Store greatest distance
		if distance > bestDistance {
			bestDistance = distance
			bestIndex = i
		}
	}

	return bestIndex, bestDistance
}

// Returns the extreme point along a direction within a polygon
func (s *Shape) getSupport(dir XY) XY {
	bestProjection := -math.MaxFloat64
	bestVertex := XY{0, 0}
	vertices := s.Vertices

	for i := 0; i < len(vertices); i++ {
		vertex := vertices[i].Position
		projection := vertex.Dot(dir)

		if projection > bestProjection {
			bestVertex = vertex
			bestProjection = projection
		}
	}

	return bestVertex
}

// Finds two polygon shapes incident face.
func findIncidentFace(v0, v1 *XY, ref, inc *Shape, index int) {
	refVerts := ref.Vertices
	incVerts := inc.Vertices
	refNorm := refVerts[index].Normal

	// Calculate normal in incident's frame of reference
	refNorm = ref.Transform.MultiplyXY(refNorm)             // To world space
	refNorm = inc.Transform.Transpose().MultiplyXY(refNorm) // To incident's model space

	// Find most anti-normal face on polygon
	incidentFace := 0
	minDot := math.MaxFloat64

	for i := 0; i < len(incVerts); i++ {
		dot := refNorm.Dot(incVerts[i].Normal)

		if dot < minDot {
			minDot = dot
			incidentFace = i
		}
	}

	// Assign face vertices for incident face
	*v0 = inc.Transform.MultiplyXY(incVerts[incidentFace].Position)
	*v0 = v0.Add(inc.Body.Position)
	incidentFace = (incidentFace + 1) % len(incVerts)
	*v1 = inc.Transform.MultiplyXY(incVerts[incidentFace].Position)
	*v1 = v1.Add(inc.Body.Position)
}
