package main

import (
	"math"
)

const earthRadiusM = 6_371_000.0 // defined by FAI sphere

// tautString runs the iterative string-tautening algorithm using great-circle
// geometry. It returns the touching-point lat/lon (degrees) for each waypoint:
// the optimal point on each cylinder boundary that minimises the total route
// length. Endpoints are placed on the near side toward their single neighbour;
// intermediate points are placed toward the foot of the perpendicular from the
// cylinder centre to the great-circle arc through the adjacent touching points.
// The algorithm iterates until the largest move is under 1 cm.
func tautStringFai(task []Waypoint) (tlat, tlon []float64) {
	n := len(task)
	tlat = make([]float64, n)
	tlon = make([]float64, n)
	for i, wp := range task {
		tlat[i], tlon[i] = wp.Lat, wp.Lon
	}

	updateEndpoints := func() {
		for _, ends := range [][2]int{{0, 1}, {n - 1, n - 2}} {
			i, j := ends[0], ends[1]
			d := haversine(task[i].Lat, task[i].Lon, tlat[j], tlon[j])
			if d > 1e-6 {
				tlat[i], tlon[i] = pointTowardFai(task[i].Lat, task[i].Lon, tlat[j], tlon[j], task[i].Radius)
			}
		}
	}

	for range 300 {
		updateEndpoints()
		maxMove := 0.0

		for i := 1; i < n-1; i++ {
			fLat, fLon := closestOnArc(tlat[i-1], tlon[i-1], tlat[i+1], tlon[i+1], task[i].Lat, task[i].Lon)
			d := haversine(task[i].Lat, task[i].Lon, fLat, fLon)
			if d < 1e-6 {
				continue // centre lies on the arc; keep current touching point
			}
			newLat, newLon := pointToward(task[i].Lat, task[i].Lon, fLat, fLon, task[i].Radius)
			move := haversine(tlat[i], tlon[i], newLat, newLon)
			if move > maxMove {
				maxMove = move
			}
			tlat[i], tlon[i] = newLat, newLon
		}
		if maxMove < 0.01 {
			break
		}
	}
	updateEndpoints()
	return tlat, tlon
}

// closestOnArc returns the point on the great-circle arc A->B closest to C,
// clamped to the arc endpoints if the perpendicular from C falls outside the arc.
func closestOnArc(aLat, aLon, bLat, bLon, cLat, cLon float64) (lat, lon float64) {
	ax, ay, az := toUnit(aLat, aLon)
	bx, by, bz := toUnit(bLat, bLon)
	cx, cy, cz := toUnit(cLat, cLon)

	// Normal to the great-circle plane through A and B.
	nx, ny, nz := cross3(ax, ay, az, bx, by, bz)
	nd := math.Sqrt(nx*nx + ny*ny + nz*nz)
	if nd < 1e-15 {
		return aLat, aLon // A and B coincide
	}
	nx, ny, nz = nx/nd, ny/nd, nz/nd

	// Closest point on the great-circle to C: project C onto the great-circle
	// plane (remove its component along the normal N), then normalise.
	nDotC := dot3(nx, ny, nz, cx, cy, cz)
	px, py, pz := cx-nDotC*nx, cy-nDotC*ny, cz-nDotC*nz
	pd := math.Sqrt(px*px + py*py + pz*pz)
	if pd < 1e-15 {
		return aLat, aLon // C is at the pole of the great-circle; undefined
	}
	px, py, pz = px/pd, py/pd, pz/pd

	// P is on the arc A→B iff (A×P)·N >= 0 and (P×B)·N >= 0.
	apx, apy, apz := cross3(ax, ay, az, px, py, pz)
	pbx, pby, pbz := cross3(px, py, pz, bx, by, bz)
	if dot3(apx, apy, apz, nx, ny, nz) >= 0 && dot3(pbx, pby, pbz, nx, ny, nz) >= 0 {
		return fromUnit(px, py, pz)
	}

	// Outside the arc: return the closer endpoint.
	if haversine(aLat, aLon, cLat, cLon) <= haversine(bLat, bLon, cLat, cLon) {
		return aLat, aLon
	}
	return bLat, bLon
}

// pointToward returns the point at distance dist (metres) from (fromLat, fromLon)
// along the great circle toward (toLat, toLon).
func pointTowardFai(fromLat, fromLon, toLat, toLon, dist float64) (lat, lon float64) {
	lat1Rad := fromLat * math.Pi / 180
	lon1Rad := fromLon * math.Pi / 180
	lat2Rad := toLat * math.Pi / 180
	lon2Rad := toLon * math.Pi / 180

	// Initial bearing.
	y := math.Sin(lon2Rad-lon1Rad) * math.Cos(lat2Rad)
	x := math.Cos(lat1Rad)*math.Sin(lat2Rad) - math.Sin(lat1Rad)*math.Cos(lat2Rad)*math.Cos(lon2Rad-lon1Rad)
	bearing := math.Atan2(y, x)

	// Destination at angular distance dist/R along bearing.
	angDist := dist / earthRadiusM
	destLatRad := math.Asin(math.Sin(lat1Rad)*math.Cos(angDist) + math.Cos(lat1Rad)*math.Sin(angDist)*math.Cos(bearing))
	destLonRad := lon1Rad + math.Atan2(math.Sin(bearing)*math.Sin(angDist)*math.Cos(lat1Rad), math.Cos(angDist)-math.Sin(lat1Rad)*math.Sin(destLatRad))
	return destLatRad * 180 / math.Pi, destLonRad * 180 / math.Pi
}

// toUnit converts lat/lon (degrees) to an ECEF unit vector.
func toUnit(lat, lon float64) (x, y, z float64) {
	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180
	return math.Cos(latRad) * math.Cos(lonRad), math.Cos(latRad) * math.Sin(lonRad), math.Sin(latRad)
}

// fromUnit converts an ECEF unit vector to lat/lon (degrees).
func fromUnit(x, y, z float64) (lat, lon float64) {
	return math.Atan2(z, math.Sqrt(x*x+y*y)) * 180 / math.Pi,
		math.Atan2(y, x) * 180 / math.Pi
}

func cross3(ax, ay, az, bx, by, bz float64) (float64, float64, float64) {
	return ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx
}

func dot3(ax, ay, az, bx, by, bz float64) float64 {
	return ax*bx + ay*by + az*bz
}

// haversine returns the great-circle distance in meters between two points
// given as (lat, lon) pairs in decimal degrees.
func Haversine(lat1, lon1, lat2, lon2 float64) float64 {
	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLat := (lat2 - lat1) * math.Pi / 180
	deltaLon := (lon2 - lon1) * math.Pi / 180

	sinDLat := math.Sin(deltaLat / 2)
	sinDLon := math.Sin(deltaLon / 2)
	a := sinDLat*sinDLat + math.Cos(lat1Rad)*math.Cos(lat2Rad)*sinDLon*sinDLon

	//return earthRadiusM * 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	// d = r*theta
	// theta = 2*arcsin(sqrt(hav(theta)))  (a = hav(theta)
	return earthRadiusM * 2 * math.Asin(math.Sqrt(a))
}
