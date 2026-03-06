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
	Index    int     // index of the fix in the fixes array at/just after achievement
	Distance float64 // meters from the previous split's crossing point; 0 for the first split
}

// ScoreResult holds the outcome of scoring a flight against a task.
type ScoreResult struct {
	Splits    []Split       // one per achieved waypoint, in task order
	SpeedTime time.Duration // elapsed time from the start to the ESS split;
	//                           falls back to start->last split if the task has no ESS
	TaskComplete bool // true if the GOAL waypoint was reached (or all waypoints
	//                              were achieved when the task has no GOAL)
	TotalOptimizedDistance        float64 // meters: shortest possible route through all cylinders
	SpeedSectionOptimizedDistance float64 // meters: optimized distance from SSS to ESS (0 if no ESS)
	DistanceMade                  float64 // meters: optimized distance covered along the task
}

// ScoreFlight scores a flight against an ordered task.
//
// Every crossing of the first waypoint is tried as a possible start. For each
// start candidate the remaining waypoints are scored greedily (earliest
// crossing wins). The attempt that completes the most waypoints wins; ties are
// broken by shortest SpeedTime.
//
// Waypoint types determine the achievement condition:
//   - WPTypeExit  (start): achieved when the pilot EXITS the cylinder (inside→outside).
//   - WPTypeNormal / WPTypeESS / WPTypeGoal: achieved when the pilot first ENTERS.
//
// If interpolate is false (default), Split.Time is the timestamp of the first
// fix that satisfies the condition. If true, it is linearly interpolated to the
// exact moment the pilot crossed the cylinder boundary.
func ScoreFlight(fixes []Fix, task []Waypoint, interpolate bool) ScoreResult {
	if len(fixes) == 0 || len(task) == 0 {
		return ScoreResult{}
	}

	startCrossings := findAllCrossings(fixes, 0, task[0], interpolate)
	if len(startCrossings) == 0 {
		return ScoreResult{}
	}

	var best []Split
	for _, start := range startCrossings {
		attempt := scoreFrom(fixes, task, start, interpolate)
		if isBetter(attempt, best) {
			best = attempt
		}
	}
	return buildResult(best, task, fixes)
}

// buildResult constructs a ScoreResult from the best split sequence and task.
func buildResult(splits []Split, task []Waypoint, fixes []Fix) ScoreResult {
	if len(splits) == 0 {
		return ScoreResult{}
	}

	taskHasESS, taskHasGoal := false, false
	for _, wp := range task {
		if wp.Type == WPTypeESS {
			taskHasESS = true
		}
		if wp.Type == WPTypeGoal {
			taskHasGoal = true
		}
	}

	startTime := splits[0].Time
	var speedTime time.Duration
	taskComplete := false

	for _, s := range splits {
		if taskHasESS && s.Waypoint.Type == WPTypeESS {
			speedTime = s.Time.Sub(startTime)
		}
		if taskHasGoal && s.Waypoint.Type == WPTypeGoal {
			taskComplete = true
		}
	}

	// Fallbacks when the task has no ESS or no GOAL.
	if !taskHasESS {
		speedTime = splits[len(splits)-1].Time.Sub(startTime)
	}
	if !taskHasGoal && len(splits) == len(task) {
		taskComplete = true
	}

	for i := 1; i < len(splits); i++ {
		prev := fixes[splits[i-1].Index]
		cur := fixes[splits[i].Index]
		splits[i].Distance = earthModel.Distance(prev.Lat, prev.Lon, cur.Lat, cur.Lon)
	}

	totalDist, speedDist := optimizedTaskDistance(task)
	distMade := computeDistanceMade(fixes, task, splits, totalDist)

	return ScoreResult{
		Splits:                        splits,
		SpeedTime:                     speedTime,
		TaskComplete:                  taskComplete,
		TotalOptimizedDistance:        totalDist,
		SpeedSectionOptimizedDistance: speedDist,
		DistanceMade:                  distMade,
	}
}

// tautString finds optimal cylinder-touching points using angular gradient
// descent. Each touching point is parameterised by its bearing from the cylinder
// centre; at every iteration each point takes a step clockwise or
// counterclockwise (whichever reduces the sum of distances to its neighbours),
// or stays put if neither direction helps. Iteration stops when the total
// distance improvement across all points in one pass falls below threshold.
func tautString(task []Waypoint) (tlat, tlon []float64) {
	n := len(task)
	if n < 2 {
		return nil, nil
	}
	tlat = make([]float64, n)
	tlon = make([]float64, n)

	// Initialise each touching point on the cylinder boundary facing toward
	// the next waypoint centre (or previous, for the last waypoint).
	for i, wp := range task {
		var dirLat, dirLon float64
		if i < n-1 {
			dirLat, dirLon = task[i+1].Lat, task[i+1].Lon
		} else {
			dirLat, dirLon = task[i-1].Lat, task[i-1].Lon
		}
		tlat[i], tlon[i] = pointToward(wp.Lat, wp.Lon, dirLat, dirLon, wp.Radius)
	}

	// segCost returns the sum of distances from point i at (lat, lon)
	// to its current adjacent touching points, using the active earth model.
	segCost := func(i int, lat, lon float64) float64 {
		cost := 0.0
		if i > 0 {
			cost += earthModel.Distance(tlat[i-1], tlon[i-1], lat, lon)
		}
		if i < n-1 {
			cost += earthModel.Distance(lat, lon, tlat[i+1], tlon[i+1])
		}
		return cost
	}

	// arcStep is the fixed arc-length step on the cylinder boundary (metres).
	// angStep for each cylinder = arcStep / radius, keeping physical step size
	// constant regardless of cylinder size.
	const arcStep = 0.5 // metres

	for range 100_000 {
		totalDistChange := 0.0

		for i := 0; i < n; i++ {
			angStep := arcStep / task[i].Radius
			_, bearing := earthModel.Inverse(task[i].Lat, task[i].Lon, tlat[i], tlon[i])
			curCost := segCost(i, tlat[i], tlon[i])

			cwLat, cwLon := earthModel.Destination(task[i].Lat, task[i].Lon, bearing+angStep, task[i].Radius)
			cwCost := segCost(i, cwLat, cwLon)

			ccwLat, ccwLon := earthModel.Destination(task[i].Lat, task[i].Lon, bearing-angStep, task[i].Radius)
			ccwCost := segCost(i, ccwLat, ccwLon)

			newLat, newLon, newCost := tlat[i], tlon[i], curCost
			if cwCost < newCost {
				newLat, newLon, newCost = cwLat, cwLon, cwCost
			}
			if ccwCost < newCost {
				newLat, newLon, newCost = ccwLat, ccwLon, ccwCost
			}

			totalDistChange += curCost - newCost
			tlat[i], tlon[i] = newLat, newLon
		}

		if totalDistChange < 1e-6 {
			break
		}
	}
	return tlat, tlon
}

// pointToward returns the point at distance dist (metres) from (fromLat, fromLon)
// toward (toLat, toLon) using the active earth model.
func pointToward(fromLat, fromLon, toLat, toLon, dist float64) (lat, lon float64) {
	_, az := earthModel.Inverse(fromLat, fromLon, toLat, toLon)
	return earthModel.Destination(fromLat, fromLon, az, dist)
}

// optimizedTaskDistance returns the length of the shortest possible route
// through all task cylinders in order, using the string-tautening algorithm.
// It also returns the speed-section distance (SSS→ESS); if the task has no
// ESS waypoint, speedSection is 0.
func optimizedTaskDistance(task []Waypoint) (total float64, speedSection float64) {
	n := len(task)
	if n < 2 {
		return 0, 0
	}
	tlat, tlon := tautString(task)

	essIdx := -1
	for i, wp := range task {
		if wp.Type == WPTypeESS {
			essIdx = i
		}
	}

	for i := 0; i < n-1; i++ {
		d := earthModel.Distance(tlat[i], tlon[i], tlat[i+1], tlon[i+1])
		total += d
		if essIdx >= 0 && i < essIdx {
			speedSection += d
		}
	}
	return total, speedSection
}

// waypointCumulativeDistances returns, for each waypoint, the cumulative
// optimized distance from the start of the task to that waypoint's optimal
// touching point. The first entry is always 0.
func waypointCumulativeDistances(task []Waypoint) []float64 {
	n := len(task)
	result := make([]float64, n)
	if n < 2 {
		return result
	}
	tlat, tlon := tautString(task)
	for i := 1; i < n; i++ {
		result[i] = result[i-1] + earthModel.Distance(tlat[i-1], tlon[i-1], tlat[i], tlon[i])
	}
	return result
}

// computeDistanceMade returns how far along the optimized task route the pilot
// got. For a complete task this equals totalDist. For an incomplete task it
// scans fixes from the last achieved split forward to find the closest approach
// to the next waypoint, then subtracts the remaining optimized distance.
func computeDistanceMade(fixes []Fix, task []Waypoint, splits []Split, totalDist float64) float64 {
	if len(splits) == len(task) {
		return totalDist
	}

	// Start scanning from the fix where the last achieved waypoint was scored.
	searchFrom := 0
	if len(splits) > 0 {
		searchFrom = splits[len(splits)-1].Index
	}

	// Index of the next (unachieved) waypoint.
	nextIdx := len(splits)
	nextWP := task[nextIdx]

	// Find the closest the pilot got to the next waypoint's center.
	minDist := math.MaxFloat64
	for i := searchFrom; i < len(fixes); i++ {
		d := earthModel.Distance(fixes[i].Lat, fixes[i].Lon, nextWP.Lat, nextWP.Lon)
		if d < minDist {
			minDist = d
		}
	}

	// Remaining distance = gap from closest point to next cylinder boundary
	// + optimized distance through the rest of the task from that waypoint.
	remaining := math.Max(0, minDist-nextWP.Radius)
	subTotal, _ := optimizedTaskDistance(task[nextIdx:])
	remaining += subTotal

	return math.Max(0, totalDist-remaining)
}

// ProgressPoint records the distance along the optimized task route achieved
// at a given point in time.
type ProgressPoint struct {
	Time     time.Time
	Progress float64 // metres along the optimized task route
}

// computeProgress returns a ProgressPoint for every valid fix at or after the
// start split, recording how far along the optimized task route the pilot has
// progressed at that moment. Returns nil if there are no splits (no valid start).
// Progress is monotonically non-decreasing: it reflects the closest approach
// to the goal achieved so far, mirroring the logic in computeDistanceMade.
func computeProgress(fixes []Fix, task []Waypoint, splits []Split, totalDist float64) []ProgressPoint {
	if len(fixes) == 0 || len(task) == 0 || len(splits) == 0 {
		return nil
	}

	// Precompute suffix optimized distances: suffixDist[i] is the optimized
	// route length from task[i] to the end of the task.
	suffixDist := make([]float64, len(task))
	for i := range task {
		if len(task)-i >= 2 {
			suffixDist[i], _ = optimizedTaskDistance(task[i:])
		}
	}

	startIdx := splits[0].Index
	result := make([]ProgressPoint, 0, len(fixes)-startIdx)
	var maxProgress float64
	// The start split (splits[0]) is already achieved; begin scanning from its fix.
	nAchieved := 1

	for i, fix := range fixes[startIdx:] {
		if !fix.Valid {
			continue
		}

		// Advance nAchieved past splits whose scoring fix is at or before this fix.
		// Adjust i back to the original fixes index for comparison.
		absIdx := startIdx + i
		for nAchieved < len(splits) && splits[nAchieved].Index <= absIdx {
			nAchieved++
		}

		var progress float64
		if nAchieved == len(task) {
			progress = totalDist
		} else {
			nextWP := task[nAchieved]
			d := earthModel.Distance(fix.Lat, fix.Lon, nextWP.Lat, nextWP.Lon)
			remaining := math.Max(0, d-nextWP.Radius) + suffixDist[nAchieved]
			progress = math.Max(0, totalDist-remaining)
		}

		if progress > maxProgress {
			maxProgress = progress
		}
		result = append(result, ProgressPoint{Time: fix.Timestamp, Progress: maxProgress})

		// Done when we reach the goal
		if maxProgress == totalDist {
			break
		}
	}

	return result
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
		// achievedIdx is already on the new side of the boundary, so
		// the next findAchievement call will wait for the next flip.
		searchFrom = achievedIdx
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
		if earthModel.Distance(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon) > wp.Radius {
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
	return findCrossing(fixes, from, wp, interpolate)
}

// findCrossing finds the first boundary crossing of wp's cylinder at or after
// from, regardless of direction. It records the initial inside/outside state
// from fixes[from] and returns the first fix where that state changes.
func findCrossing(fixes []Fix, from int, wp Waypoint, interpolate bool) (Split, bool, int) {
	if from >= len(fixes) {
		return Split{}, false, from
	}
	startInside := earthModel.Distance(fixes[from].Lat, fixes[from].Lon, wp.Lat, wp.Lon) <= wp.Radius
	for i := from + 1; i < len(fixes); i++ {
		d := earthModel.Distance(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon)
		if (d <= wp.Radius) != startInside {
			t := fixes[i].Timestamp
			if interpolate {
				dPrev := earthModel.Distance(fixes[i-1].Lat, fixes[i-1].Lon, wp.Lat, wp.Lon)
				t = interpolateCrossing(fixes[i-1], fixes[i], dPrev, d, wp.Radius)
			}
			return Split{Waypoint: wp, Time: t, Index: i}, true, i
		}
	}
	return Split{}, false, from
}

// findEntry finds the first fix where the pilot is inside the cylinder.
// With interpolate=true, the time is walked back to the boundary crossing.
func findEntry(fixes []Fix, from int, wp Waypoint, interpolate bool) (Split, bool, int) {
	for i := from; i < len(fixes); i++ {
		d := earthModel.Distance(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon)
		if d <= wp.Radius {
			t := fixes[i].Timestamp
			if interpolate && i > 0 {
				dPrev := earthModel.Distance(fixes[i-1].Lat, fixes[i-1].Lon, wp.Lat, wp.Lon)
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
		d := earthModel.Distance(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon)
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
		d := earthModel.Distance(fixes[i].Lat, fixes[i].Lon, wp.Lat, wp.Lon)
		if d > wp.Radius {
			t := fixes[i].Timestamp
			if interpolate {
				dPrev := earthModel.Distance(fixes[i-1].Lat, fixes[i-1].Lon, wp.Lat, wp.Lon)
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
	// frac is the fractional position along a->b where distance equals radius.
	frac := (radius - dA) / (dB - dA)
	dt := b.Timestamp.Sub(a.Timestamp)
	return a.Timestamp.Add(time.Duration(float64(dt) * frac))
}
