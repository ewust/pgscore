package main

import (
	"fmt"
	"os"
	"time"
)

func main() {
	if len(os.Args) < 2 {
		fmt.Fprintf(os.Stderr, "Usage: %s <flight.igc>\n", os.Args[0])
		os.Exit(1)
	}

	flight, err := ParseIGC(os.Args[1])
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error parsing IGC file: %v\n", err)
		os.Exit(1)
	}

	fmt.Printf("=== IGC Flight Summary ===\n")
	fmt.Printf("Date:   %s\n", flight.Date.Format("2006-01-02"))
	if flight.PilotName != "" {
		fmt.Printf("Pilot:  %s\n", flight.PilotName)
	}
	if flight.GliderType != "" {
		fmt.Printf("Glider: %s\n", flight.GliderType)
	}
	if flight.GliderID != "" {
		fmt.Printf("Reg:    %s\n", flight.GliderID)
	}

	fmt.Printf("\n--- GPS Fixes: %d ---\n", len(flight.Fixes))
	if len(flight.Fixes) > 0 {
		first := flight.Fixes[0]
		last := flight.Fixes[len(flight.Fixes)-1]
		duration := last.Timestamp.Sub(first.Timestamp).Round(time.Second)

		fmt.Printf("First:    %s  %s\n", first.Timestamp.Format("15:04:05 UTC"), formatFix(first))
		fmt.Printf("Last:     %s  %s\n", last.Timestamp.Format("15:04:05 UTC"), formatFix(last))
		fmt.Printf("Duration: %s\n", formatDuration(duration))
	}

	if len(flight.Task) > 0 {
		fmt.Printf("\n--- Declared Task: %d waypoints ---\n", len(flight.Task))
		for i, wp := range flight.Task {
			label := taskLabel(i, len(flight.Task))
			fmt.Printf("  %d  %-8s  lat=%9.5f  lon=%10.5f  %s\n",
				i+1, label, wp.Lat, wp.Lon, wp.Name)
		}
	} else {
		fmt.Printf("\nNo task declared in IGC file.\n")
	}
}

func formatFix(f Fix) string {
	validity := "valid"
	if !f.Valid {
		validity = "INVALID"
	}
	return fmt.Sprintf("lat=%9.5f  lon=%10.5f  pressAlt=%5dm  gnssAlt=%5dm  [%s]",
		f.Lat, f.Lon, f.PressureAlt, f.GNSSAlt, validity)
}

// taskLabel returns a human-readable role label for the i-th waypoint of total.
// Standard IGC task order: Takeoff, Start, TP1…TPn, Finish, Landing.
func taskLabel(i, total int) string {
	switch {
	case i == 0:
		return "Takeoff"
	case i == total-1:
		return "Landing"
	case i == 1:
		return "Start"
	case i == total-2:
		return "Finish"
	default:
		return fmt.Sprintf("TP%d", i-1)
	}
}

// formatDuration renders a duration as H:MM:SS.
func formatDuration(d time.Duration) string {
	d = d.Round(time.Second)
	h := int(d.Hours())
	m := int(d.Minutes()) % 60
	s := int(d.Seconds()) % 60
	return fmt.Sprintf("%d:%02d:%02d", h, m, s)
}
