package main

import "math"

// FAI sphere radius, as defined by the FAI Sporting Code.
const earthRadiusM = 6_371_000.0

// faiDistance returns the great-circle distance in metres between two lat/lon
// points (decimal degrees) on the FAI sphere, using haversine formula
func faiDistance(lat1, lon1, lat2, lon2 float64) float64 {
	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLat := (lat2 - lat1) * math.Pi / 180
	deltaLon := (lon2 - lon1) * math.Pi / 180

	sinDLat := math.Sin(deltaLat / 2)
	sinDLon := math.Sin(deltaLon / 2)
	a := sinDLat*sinDLat + math.Cos(lat1Rad)*math.Cos(lat2Rad)*sinDLon*sinDLon
	return earthRadiusM * 2 * math.Asin(math.Sqrt(a))
}

// faiBearing returns the initial forward bearing in radians (0 = north, clockwise)
// from (lat1, lon1) to (lat2, lon2) on the FAI sphere.
func faiBearing(lat1, lon1, lat2, lon2 float64) float64 {
	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLon := (lon2 - lon1) * math.Pi / 180
	y := math.Sin(deltaLon) * math.Cos(lat2Rad)
	x := math.Cos(lat1Rad)*math.Sin(lat2Rad) - math.Sin(lat1Rad)*math.Cos(lat2Rad)*math.Cos(deltaLon)
	return math.Atan2(y, x)
}

// faiInverse returns the great-circle distance (metres) and initial forward
// bearing (radians) from point 1 to point 2 on the FAI sphere.
func faiInverse(lat1, lon1, lat2, lon2 float64) (distance, azimuth float64) {
	return faiDistance(lat1, lon1, lat2, lon2), faiBearing(lat1, lon1, lat2, lon2)
}

// faiDestination returns the point reached by travelling dist metres from
// (lat, lon) along azimuth (radians, 0 = north, clockwise) on the FAI sphere.
func faiDestination(lat, lon, azimuth, dist float64) (destLat, destLon float64) {
	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180
	angDist := dist / earthRadiusM
	destLatRad := math.Asin(math.Sin(latRad)*math.Cos(angDist) + math.Cos(latRad)*math.Sin(angDist)*math.Cos(azimuth))
	destLonRad := lonRad + math.Atan2(math.Sin(azimuth)*math.Sin(angDist)*math.Cos(latRad), math.Cos(angDist)-math.Sin(latRad)*math.Sin(destLatRad))
	return destLatRad * 180 / math.Pi, destLonRad * 180 / math.Pi
}

// faiClosestOnArc returns the point on the great-circle arc A->B closest to C,
// clamped to the arc endpoints, using ECEF unit-vector geometry.
func faiClosestOnArc(aLat, aLon, bLat, bLon, cLat, cLon float64) (lat, lon float64) {
	ax, ay, az := toUnit(aLat, aLon)
	bx, by, bz := toUnit(bLat, bLon)
	cx, cy, cz := toUnit(cLat, cLon)

	// Normal to the great-circle plane through A and B.
	nx, ny, nz := cross3(ax, ay, az, bx, by, bz)
	nd := math.Sqrt(nx*nx + ny*ny + nz*nz)
	if nd < 1e-15 {
		return aLat, aLon
	}
	nx, ny, nz = nx/nd, ny/nd, nz/nd

	// Project C onto the great-circle plane and normalise.
	nDotC := dot3(nx, ny, nz, cx, cy, cz)
	px, py, pz := cx-nDotC*nx, cy-nDotC*ny, cz-nDotC*nz
	pd := math.Sqrt(px*px + py*py + pz*pz)
	if pd < 1e-15 {
		return aLat, aLon
	}
	px, py, pz = px/pd, py/pd, pz/pd

	// P is on arc A->B iff (A×P)·N >= 0 and (P×B)·N >= 0.
	apx, apy, apz := cross3(ax, ay, az, px, py, pz)
	pbx, pby, pbz := cross3(px, py, pz, bx, by, bz)
	if dot3(apx, apy, apz, nx, ny, nz) >= 0 && dot3(pbx, pby, pbz, nx, ny, nz) >= 0 {
		return fromUnit(px, py, pz)
	}

	if faiDistance(aLat, aLon, cLat, cLon) <= faiDistance(bLat, bLon, cLat, cLon) {
		return aLat, aLon
	}
	return bLat, bLon
}

// ECEF unit-vector helpers (used by faiClosestOnArc).

func toUnit(lat, lon float64) (x, y, z float64) {
	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180
	return math.Cos(latRad) * math.Cos(lonRad), math.Cos(latRad) * math.Sin(lonRad), math.Sin(latRad)
}

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

// FAIModel implements EarthModel using great-circle geometry on the FAI sphere.
type FAIModel struct{}

func (FAIModel) Distance(lat1, lon1, lat2, lon2 float64) float64 {
	return faiDistance(lat1, lon1, lat2, lon2)
}

func (FAIModel) Inverse(lat1, lon1, lat2, lon2 float64) (float64, float64) {
	return faiInverse(lat1, lon1, lat2, lon2)
}

func (FAIModel) Destination(lat, lon, azimuth, dist float64) (float64, float64) {
	return faiDestination(lat, lon, azimuth, dist)
}

func (FAIModel) ClosestOnPath(aLat, aLon, bLat, bLon, cLat, cLon float64) (float64, float64) {
	return faiClosestOnArc(aLat, aLon, bLat, bLon, cLat, cLon)
}
