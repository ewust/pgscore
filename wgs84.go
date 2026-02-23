package main

import "math"

// WGS84 reference ellipsoid constants.
const (
	wgs84A = 6_378_137.0              // semi-major axis, metres
	wgs84F = 1.0 / 298.257_223_563    // flattening
	wgs84B = wgs84A * (1 - wgs84F)    // semi-minor axis, metres
)

// geodesicInverse solves Vincenty's inverse problem on the WGS84 ellipsoid:
// given two lat/lon points (decimal degrees), it returns the geodesic distance
// in metres and the initial forward azimuth in radians (0 = north, clockwise).
// Returns (0, 0) for coincident points.
func geodesicInverse(lat1, lon1, lat2, lon2 float64) (distance, azimuth float64) {
	lat1Rad := lat1 * math.Pi / 180
	lon1Rad := lon1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	lon2Rad := lon2 * math.Pi / 180

	// Reduced latitudes.
	u1 := math.Atan((1 - wgs84F) * math.Tan(lat1Rad))
	u2 := math.Atan((1 - wgs84F) * math.Tan(lat2Rad))
	L := lon2Rad - lon1Rad

	sinU1, cosU1 := math.Sin(u1), math.Cos(u1)
	sinU2, cosU2 := math.Sin(u2), math.Cos(u2)

	lambda := L
	var sinLambda, cosLambda float64
	var sinSigma, cosSigma, sigma float64
	var sinAlpha, cosSqAlpha, cos2SigmaM float64

	for range 100 {
		sinLambda, cosLambda = math.Sin(lambda), math.Cos(lambda)
		crossTrack := cosU2 * sinLambda
		alongTrack := cosU1*sinU2 - sinU1*cosU2*cosLambda
		sinSigma = math.Sqrt(crossTrack*crossTrack + alongTrack*alongTrack)
		if sinSigma == 0 {
			return 0, 0 // coincident points
		}
		cosSigma = sinU1*sinU2 + cosU1*cosU2*cosLambda
		sigma = math.Atan2(sinSigma, cosSigma)
		sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma
		cosSqAlpha = 1 - sinAlpha*sinAlpha
		if cosSqAlpha == 0 {
			cos2SigmaM = 0 // equatorial line
		} else {
			cos2SigmaM = cosSigma - 2*sinU1*sinU2/cosSqAlpha
		}
		C := wgs84F / 16 * cosSqAlpha * (4 + wgs84F*(4-3*cosSqAlpha))
		lambdaPrev := lambda
		lambda = L + (1-C)*wgs84F*sinAlpha*(sigma+C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)))
		if math.Abs(lambda-lambdaPrev) < 1e-12 {
			break
		}
	}

	uSq := cosSqAlpha * (wgs84A*wgs84A - wgs84B*wgs84B) / (wgs84B * wgs84B)
	A := 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)))
	B := uSq / 1024 * (256 + uSq*(-128+uSq*(74-47*uSq)))
	deltaSigma := B * sinSigma * (cos2SigmaM + B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
		B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)))

	distance = wgs84B * A * (sigma - deltaSigma)
	azimuth = math.Atan2(cosU2*sinLambda, cosU1*sinU2-sinU1*cosU2*cosLambda)
	return distance, azimuth
}

// geodesicDistance returns the geodesic distance in metres between two lat/lon
// points (decimal degrees) on the WGS84 ellipsoid.
func geodesicDistance(lat1, lon1, lat2, lon2 float64) float64 {
	d, _ := geodesicInverse(lat1, lon1, lat2, lon2)
	return d
}

// geodesicDestination returns the lat/lon of the point reached by starting at
// (lat, lon) and travelling dist metres along the WGS84 geodesic at the given
// initial forward azimuth (radians, 0 = north, clockwise), using Vincenty's
// direct formula.
func geodesicDestination(lat, lon, azimuth, dist float64) (destLat, destLon float64) {
	if dist == 0 {
		return lat, lon
	}
	latRad := lat * math.Pi / 180
	lonRad := lon * math.Pi / 180

	sinAz, cosAz := math.Sin(azimuth), math.Cos(azimuth)
	tanU1 := (1 - wgs84F) * math.Tan(latRad)
	cosU1 := 1 / math.Sqrt(1+tanU1*tanU1)
	sinU1 := tanU1 * cosU1

	sigma1 := math.Atan2(tanU1, cosAz)
	sinAlpha := cosU1 * sinAz
	cosSqAlpha := 1 - sinAlpha*sinAlpha
	uSq := cosSqAlpha * (wgs84A*wgs84A - wgs84B*wgs84B) / (wgs84B * wgs84B)
	A := 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)))
	B := uSq / 1024 * (256 + uSq*(-128+uSq*(74-47*uSq)))

	sigma := dist / (wgs84B * A)
	var cos2SigmaM, sinSigma, cosSigma float64
	for range 100 {
		cos2SigmaM = math.Cos(2*sigma1 + sigma)
		sinSigma = math.Sin(sigma)
		cosSigma = math.Cos(sigma)
		deltaSigma := B * sinSigma * (cos2SigmaM + B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
			B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)))
		sigmaPrev := sigma
		sigma = dist/(wgs84B*A) + deltaSigma
		if math.Abs(sigma-sigmaPrev) < 1e-12 {
			break
		}
	}

	tmp := sinU1*sinSigma - cosU1*cosSigma*cosAz
	destLatRad := math.Atan2(sinU1*cosSigma+cosU1*sinSigma*cosAz,
		(1-wgs84F)*math.Sqrt(sinAlpha*sinAlpha+tmp*tmp))
	lam := math.Atan2(sinSigma*sinAz, cosU1*cosSigma-sinU1*sinSigma*cosAz)
	C := wgs84F / 16 * cosSqAlpha * (4 + wgs84F*(4-3*cosSqAlpha))
	L := lam - (1-C)*wgs84F*sinAlpha*(sigma+C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)))

	return destLatRad * 180 / math.Pi, (lonRad+L)*180/math.Pi
}

// closestOnGeodesic returns the point on the WGS84 geodesic arc from A to B
// closest to C, clamped to the arc endpoints. It uses ternary search over the
// arc length, which is valid because distance from C to points along a geodesic
// is unimodal on an ellipsoid.
func closestOnGeodesic(aLat, aLon, bLat, bLon, cLat, cLon float64) (lat, lon float64) {
	totalDist, az := geodesicInverse(aLat, aLon, bLat, bLon)
	if totalDist < 1e-6 {
		return aLat, aLon
	}

	distToC := func(s float64) float64 {
		pLat, pLon := geodesicDestination(aLat, aLon, az, s)
		return geodesicDistance(pLat, pLon, cLat, cLon)
	}

	lo, hi := 0.0, totalDist
	for range 52 {
		m1 := lo + (hi-lo)/3
		m2 := hi - (hi-lo)/3
		if distToC(m1) < distToC(m2) {
			hi = m2
		} else {
			lo = m1
		}
	}
	return geodesicDestination(aLat, aLon, az, (lo+hi)/2)
}
