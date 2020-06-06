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
	Vertices  []*Vertex // Polygon shape vertices position and normals verts (just used for polygon shapes).
}

// Finds polygon shapes axis least penetration.
func findAxisLeastPenetration(a, b *Shape) (int, float64) {
	bestDistance := -math.MaxFloat64
	bestIndex := 0

	vertsA := a.Vertices

	for i := 0; i < len(vertsA); i++ {
		// Retrieve a face normal from A shape
		normal := vertsA[i].Normal
		transNormal := mat2MultiplyXY(a.Transform, normal)

		// Transform face normal into B shape's model space
		buT := mat2Transpose(b.Transform)
		normal = mat2MultiplyXY(buT, transNormal)

		// Retrieve support point from B shape along -n
		support := b.getSupport(XY{-normal.X, -normal.Y})

		// Retrieve vertex on face from A shape, transform into B shape's model space
		vertex := vertsA[i].Position
		vertex = mat2MultiplyXY(a.Transform, vertex)
		vertex = xyAdd(vertex, a.Body.Position)
		vertex = xySubtract(vertex, b.Body.Position)
		vertex = mat2MultiplyXY(buT, vertex)

		// Compute penetration distance in B shape's model space
		distance := mathDot(normal, xySubtract(support, vertex))

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
		projection := mathDot(vertex, dir)

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
	refNorm = mat2MultiplyXY(ref.Transform, refNorm)                // To world space
	refNorm = mat2MultiplyXY(mat2Transpose(inc.Transform), refNorm) // To incident's model space

	// Find most anti-normal face on polygon
	incidentFace := 0
	minDot := math.MaxFloat64

	for i := 0; i < len(incVerts); i++ {
		dot := mathDot(refNorm, incVerts[i].Normal)

		if dot < minDot {
			minDot = dot
			incidentFace = i
		}
	}

	// Assign face vertices for incident face
	*v0 = mat2MultiplyXY(inc.Transform, incVerts[incidentFace].Position)
	*v0 = xyAdd(*v0, inc.Body.Position)
	incidentFace = (incidentFace + 1) % len(incVerts)
	*v1 = mat2MultiplyXY(inc.Transform, incVerts[incidentFace].Position)
	*v1 = xyAdd(*v1, inc.Body.Position)
}
