package main

import (
	"math"
	"time"
)

// Split records the moment a task waypoint was achieved during a flight.
type Split struct {
	Waypoint Waypoint  // the waypoint that was achieved
	Time     time.Time // time of achievement: the triggering fix's timestamp, or
	//                    interpolated to the boundary if ScoreFlight was called with interpolate=true
	Index int // index of the fix in the fixes array at/just after achievement
}

// ScoreFlight scores a flight against an ordered task.
//
// It walks through the task waypoints in order, finding the time when each
// cylinder was achieved. It stops early if the pilot did not complete the full
// task, returning only the achieved splits.
//
// Waypoint types determine the achievement condition:
//   - WPTypeExit  (start): achieved when the pilot EXITS the cylinder (inside→outside).
//   - WPTypeNormal / WPTypeESS / WPTypeGoal: achieved when the pilot first ENTERS.
//
// If interpolate is false (default), Split.Time is the timestamp of the first
// fix that satisfies the condition. If true, it is linearly interpolated to the
// exact moment the pilot crossed the cylinder boundary.
func ScoreFlight(fixes []Fix, task []Waypoint, interpolate bool) []Split {
	if len(fixes) == 0 || len(task) == 0 {
		return nil
	}

	var splits []Split
	searchFrom := 0

	for _, wp := range task {
		split, found, next := findAchievement(fixes, searchFrom, wp, interpolate)
		if !found {
			break
		}
		splits = append(splits, split)
		searchFrom = next
	}

	return splits
}

// findAchievement dispatches to the correct detector based on waypoint type.
func findAchievement(fixes []Fix, from int, wp Waypoint, interpolate bool) (split Split, found bool, next int) {
	if wp.Type == WPTypeExit {
		return findExit(fixes, from, wp, interpolate)
	}
	return findEntry(fixes, from, wp, interpolate)
}

// findEntry finds the first fix where the pilot is inside the cylinder.
// With interpolate=true, the time is walked back to the boundary crossing.
func findEntry(fixes []Fix, from int, wp Waypoint, interpolate bool) (Split, bool, int) {
	for i := from; i < len(fixes); i++ {
		d := haversine(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon)
		if d <= wp.Radius {
			t := fixes[i].Timestamp
			if interpolate && i > 0 {
				dPrev := haversine(fixes[i-1].Lat, fixes[i-1].Lon, wp.Lat, wp.Lon)
				if dPrev > wp.Radius {
					t = interpolateCrossing(fixes[i-1], fixes[i], dPrev, d, wp.Radius)
				}
			}
			return Split{Waypoint: wp, Time: t, Index: i}, true, i
		}
	}
	return Split{}, false, from
}

// findExit finds when the pilot exits the cylinder (inside→outside transition).
// The pilot must first be observed inside the cylinder, then leave it.
// With interpolate=true, the time is pinpointed to the boundary crossing.
func findExit(fixes []Fix, from int, wp Waypoint, interpolate bool) (Split, bool, int) {
	// Scan forward to find the first fix inside the cylinder.
	insideFrom := -1
	for i := from; i < len(fixes); i++ {
		d := haversine(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon)
		if d <= wp.Radius {
			insideFrom = i
			break
		}
	}
	if insideFrom < 0 {
		return Split{}, false, from // never entered the cylinder
	}

	// From the first inside fix, find the first fix outside (the exit).
	for i := insideFrom + 1; i < len(fixes); i++ {
		d := haversine(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon)
		if d > wp.Radius {
			t := fixes[i].Timestamp
			if interpolate {
				dPrev := haversine(fixes[i-1].Lat, fixes[i-1].Lon, wp.Lat, wp.Lon)
				t = interpolateCrossing(fixes[i-1], fixes[i], dPrev, d, wp.Radius)
			}
			return Split{Waypoint: wp, Time: t, Index: i}, true, i
		}
	}
	return Split{}, false, from // entered but never exited
}

// interpolateCrossing linearly interpolates the time of cylinder-boundary crossing
// between fix a (distance dA from center) and fix b (distance dB from center).
// Exactly one of dA, dB is inside the cylinder (< radius); the other is outside.
func interpolateCrossing(a, b Fix, dA, dB, radius float64) time.Time {
	// frac is the fractional position along a→b where distance equals radius.
	frac := (radius - dA) / (dB - dA)
	dt := b.Timestamp.Sub(a.Timestamp)
	return a.Timestamp.Add(time.Duration(float64(dt) * frac))
}

// haversine returns the great-circle distance in meters between two points
// given as (lat, lon) pairs in decimal degrees.
func haversine(lat1, lon1, lat2, lon2 float64) float64 {
	const earthRadiusM = 6_371_000

	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLat := (lat2 - lat1) * math.Pi / 180
	deltaLon := (lon2 - lon1) * math.Pi / 180

	sinDLat := math.Sin(deltaLat / 2)
	sinDLon := math.Sin(deltaLon / 2)
	a := sinDLat*sinDLat + math.Cos(lat1Rad)*math.Cos(lat2Rad)*sinDLon*sinDLon

	return earthRadiusM * 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
}
