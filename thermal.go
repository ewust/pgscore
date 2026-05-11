package main

import (
	"bufio"
	"fmt"
	"math"
	"os"
	"time"
)

const thermalNoiseThreshold = 10 // meters

// Thermal represents a single detected thermal.
type Thermal struct {
	StartIdx      int
	EndIdx        int
	StartTime     time.Time
	EndTime       time.Time
	StartAlt      int
	EndAlt        int
	PeakClimbRate float64 // m/s, maximum climb rate between any two consecutive fixes
}

// Duration returns the thermal duration.
func (t Thermal) Duration() time.Duration {
	return t.EndTime.Sub(t.StartTime)
}

// AltGain returns the altitude gained in the thermal.
func (t Thermal) AltGain() int {
	return t.EndAlt - t.StartAlt
}

// ClimbRate returns the average climb rate in m/s.
func (t Thermal) ClimbRate() float64 {
	secs := t.Duration().Seconds()
	if secs <= 0 {
		return 0
	}
	return float64(t.AltGain()) / secs
}

// GetThermals detects thermals in a tracklog using a noise threshold of
// thermalNoiseThreshold meters. A thermal begins when altitude rises more than
// the threshold above a local minimum, and ends when altitude falls more than
// the threshold below the subsequent local maximum.
func GetThermals(fixes []Fix) []Thermal {
	if len(fixes) == 0 {
		return nil
	}

	var thermals []Thermal

	// State: searching for thermal start.
	searching := true
	baseIdx := 0    // index of local minimum (thermal start candidate)
	peakIdx := 0    // index of local maximum (thermal end candidate)
	maxClimbRate := 0.0

	for i, fix := range fixes {
		alt := fix.GNSSAlt

		if searching {
			// Track the local minimum.
			if alt <= fixes[baseIdx].GNSSAlt {
				baseIdx = i
			}
			// If we've risen thermalNoiseThreshold above the base, thermal started.
			if alt >= fixes[baseIdx].GNSSAlt+thermalNoiseThreshold {
				searching = false
				peakIdx = i
				maxClimbRate = 0
			}
		} else {
			// Track per-fix climb rate.
			if i > 0 {
				dt := fix.Timestamp.Sub(fixes[i-1].Timestamp).Seconds()
				if dt > 0 {
					rate := float64(alt-fixes[i-1].GNSSAlt) / dt
					if rate > maxClimbRate {
						maxClimbRate = rate
					}
				}
			}
			// In a thermal: track the local maximum.
			if alt >= fixes[peakIdx].GNSSAlt {
				peakIdx = i
			}
			// If we've dropped thermalNoiseThreshold below the peak, thermal ended.
			if alt <= fixes[peakIdx].GNSSAlt-thermalNoiseThreshold {
				thermals = append(thermals, Thermal{
					StartIdx:      baseIdx,
					EndIdx:        peakIdx,
					StartTime:     fixes[baseIdx].Timestamp,
					EndTime:       fixes[peakIdx].Timestamp,
					StartAlt:      fixes[baseIdx].GNSSAlt,
					EndAlt:        fixes[peakIdx].GNSSAlt,
					PeakClimbRate: maxClimbRate,
				})
				// Start searching again from the current peak.
				searching = true
				baseIdx = peakIdx
				peakIdx = i
			}
		}
	}

	return thermals
}

const vsdistCutoff = 15.0 // m/s

// writeVSDist is the shared core: it accumulates vertical-speed buckets from
// the provided index pairs and writes a TSV to filename.
func writeVSDist(fixes []Fix, pairs [][2]int, filename string, step float64) error {
	minB := int(math.Round(-vsdistCutoff / step))
	maxB := int(math.Round(vsdistCutoff / step))
	counts := make(map[int]float64)

	for _, p := range pairs {
		prev, cur := p[0], p[1]
		dt := fixes[cur].Timestamp.Sub(fixes[prev].Timestamp).Seconds()
		if dt <= 0 {
			continue
		}
		vs := float64(fixes[cur].GNSSAlt-fixes[prev].GNSSAlt) / dt
		vs = math.Max(-vsdistCutoff, math.Min(vsdistCutoff, vs))
		counts[int(math.Round(vs/step))] += dt
	}

	if len(counts) == 0 {
		return nil
	}

	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	w := bufio.NewWriter(f)
	fmt.Fprintf(w, "vertical_speed_ms\ttime_seconds\n")
	for b := minB; b <= maxB; b++ {
		fmt.Fprintf(w, "%.4f\t%.1f\n", float64(b)*step, counts[b])
	}
	return w.Flush()
}

// WriteVSDist computes a vertical speed distribution over all consecutive fix
// pairs in the flight and writes a TSV to filename.
func WriteVSDist(fixes []Fix, filename string, step float64) error {
	if len(fixes) < 2 || step <= 0 {
		return nil
	}
	pairs := make([][2]int, len(fixes)-1)
	for i := range pairs {
		pairs[i] = [2]int{i, i + 1}
	}
	return writeVSDist(fixes, pairs, filename, step)
}

// WriteThermalVSDist computes a vertical speed distribution restricted to fix
// pairs that fall within detected thermal windows and writes a TSV to filename.
func WriteThermalVSDist(fixes []Fix, thermals []Thermal, filename string, step float64) error {
	if len(fixes) < 2 || step <= 0 {
		return nil
	}
	var pairs [][2]int
	for _, t := range thermals {
		for i := t.StartIdx + 1; i <= t.EndIdx; i++ {
			pairs = append(pairs, [2]int{i - 1, i})
		}
	}
	return writeVSDist(fixes, pairs, filename, step)
}

// PrintThermals prints a summary of detected thermals to stdout.
func PrintThermals(thermals []Thermal) {
	if len(thermals) == 0 {
		fmt.Println("No thermals detected.")
		return
	}
	fmt.Printf("\n--- Thermals (%d) ---\n", len(thermals))
	fmt.Printf("%-4s  %-10s  %-10s  %-8s  %-10s  %-10s\n",
		"#", "Start", "Duration", "Gain(m)", "Avg(m/s)", "Peak(m/s)")
	for i, t := range thermals {
		fmt.Printf("%-4d  %-10s  %-10s  %-8d  +%-9.2f  +%-9.2f\n",
			i+1,
			t.StartTime.Format("15:04:05"),
			formatDuration(t.Duration()),
			t.AltGain(),
			t.ClimbRate(),
			t.PeakClimbRate,
		)
	}
}
