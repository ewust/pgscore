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
// Every crossing of the first waypoint is tried as a possible start. For each
// start candidate the remaining waypoints are scored greedily (earliest
// crossing wins). The attempt that completes the most waypoints wins; ties are
// broken by shortest total time.
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

	startCrossings := findAllCrossings(fixes, 0, task[0], interpolate)
	if len(startCrossings) == 0 {
		return nil
	}

	var best []Split
	for _, start := range startCrossings {
		attempt := scoreFrom(fixes, task, start, interpolate)
		if isBetter(attempt, best) {
			best = attempt
		}
	}
	return best
}

// scoreFrom greedily scores task[1:] starting from the given start split,
// taking the earliest crossing of each subsequent waypoint in order.
func scoreFrom(fixes []Fix, task []Waypoint, start Split, interpolate bool) []Split {
	splits := []Split{start}
	searchFrom := start.Index
	for _, wp := range task[1:] {
		split, found, next := findAchievement(fixes, searchFrom, wp, interpolate)
		if !found {
			break
		}
		splits = append(splits, split)
		searchFrom = next
	}
	return splits
}

// findAllCrossings returns every achievement of wp in fixes[from:].
// After each crossing, the search advances past the pilot's exit from the
// cylinder so the next distinct crossing can be found.
func findAllCrossings(fixes []Fix, from int, wp Waypoint, interpolate bool) []Split {
	var all []Split
	searchFrom := from
	for {
		split, found, achievedIdx := findAchievement(fixes, searchFrom, wp, interpolate)
		if !found {
			break
		}
		all = append(all, split)
		// Advance past this crossing so the next call finds a new one.
		// For EXIT: achievedIdx is the first fix outside; findExit naturally
		//           scans forward from there to find the next re-entry.
		// For ENTRY: achievedIdx is the first fix inside; advance to the
		//            first fix outside before searching again.
		if wp.Type == WPTypeExit {
			searchFrom = achievedIdx
		} else {
			searchFrom = firstOutsideIdx(fixes, achievedIdx, wp)
		}
		if searchFrom >= len(fixes) {
			break
		}
	}
	return all
}

// firstOutsideIdx returns the index of the first fix outside wp's cylinder
// at or after from. Returns len(fixes) if the pilot never leaves.
func firstOutsideIdx(fixes []Fix, from int, wp Waypoint) int {
	for i := from; i < len(fixes); i++ {
		if haversine(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon) > wp.Radius {
			return i
		}
	}
	return len(fixes)
}

// isBetter reports whether candidate is a better result than current:
// more waypoints completed wins; ties break on shorter total time.
func isBetter(candidate, current []Split) bool {
	if len(candidate) != len(current) {
		return len(candidate) > len(current)
	}
	if len(candidate) == 0 {
		return false
	}
	candidateDur := candidate[len(candidate)-1].Time.Sub(candidate[0].Time)
	currentDur := current[len(current)-1].Time.Sub(current[0].Time)
	return candidateDur < currentDur
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
