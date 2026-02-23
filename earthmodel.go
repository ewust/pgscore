package main

// EarthModel defines the geodetic primitives needed for route optimisation
// and cylinder-crossing detection. Two implementations are provided:
// WGS84Model (default, accurate ellipsoid) and FAIModel (spherical, per FAI rules).
type EarthModel interface {
	// Distance returns the distance in metres between two lat/lon points
	// (decimal degrees).
	Distance(lat1, lon1, lat2, lon2 float64) float64

	// Inverse returns the distance (metres) and initial forward azimuth
	// (radians, 0 = north, clockwise) from point 1 to point 2.
	Inverse(lat1, lon1, lat2, lon2 float64) (distance, azimuth float64)

	// Destination returns the point reached by travelling dist metres from
	// (lat, lon) along the given azimuth (radians, 0 = north, clockwise).
	Destination(lat, lon, azimuth, dist float64) (destLat, destLon float64)

	// ClosestOnPath returns the point on the geodesic arc from A to B
	// closest to C, clamped to the arc endpoints.
	ClosestOnPath(aLat, aLon, bLat, bLon, cLat, cLon float64) (lat, lon float64)
}

// earthModel is the active geodetic model, set once at startup from the
// --earth-model flag. Defaults to WGS84.
var earthModel EarthModel = WGS84Model{}
