package main

import (
	"fmt"
	"time"
)

const thermalNoiseThreshold = 10 // meters

// Thermal represents a single detected thermal.
type Thermal struct {
	StartIdx  int
	EndIdx    int
	StartTime time.Time
	EndTime   time.Time
	StartAlt  int
	EndAlt    int
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
	baseIdx := 0 // index of local minimum (thermal start candidate)
	peakIdx := 0 // index of local maximum (thermal end candidate)

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
			}
		} else {
			// In a thermal: track the local maximum.
			if alt >= fixes[peakIdx].GNSSAlt {
				peakIdx = i
			}
			// If we've dropped thermalNoiseThreshold below the peak, thermal ended.
			if alt <= fixes[peakIdx].GNSSAlt-thermalNoiseThreshold {
				thermals = append(thermals, Thermal{
					StartIdx:  baseIdx,
					EndIdx:    peakIdx,
					StartTime: fixes[baseIdx].Timestamp,
					EndTime:   fixes[peakIdx].Timestamp,
					StartAlt:  fixes[baseIdx].GNSSAlt,
					EndAlt:    fixes[peakIdx].GNSSAlt,
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

// PrintThermals prints a summary of detected thermals to stdout.
func PrintThermals(thermals []Thermal) {
	if len(thermals) == 0 {
		fmt.Println("No thermals detected.")
		return
	}
	fmt.Printf("\n--- Thermals (%d) ---\n", len(thermals))
	fmt.Printf("%-4s  %-10s  %-10s  %-8s  %-6s\n",
		"#", "Start", "Duration", "Gain(m)", "Avg(m/s)")
	for i, t := range thermals {
		fmt.Printf("%-4d  %-10s  %-10s  %-8d  +%.2f\n",
			i+1,
			t.StartTime.Format("15:04:05"),
			formatDuration(t.Duration()),
			t.AltGain(),
			t.ClimbRate(),
		)
	}
}
