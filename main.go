package main

import (
	"flag"
	"fmt"
	"os"
	"time"
)

func main() {
	taskFile := flag.String("task", "", "task definition file (e.g. inandout.txt)")
	waypointsFile := flag.String("waypoints", "", "OziExplorer waypoints file (.wpt)")
	interpolate := flag.Bool("interpolate", false, "interpolate crossing times to the cylinder boundary (default: use first qualifying fix timestamp)")
	htmlFile := flag.String("html", "", "write a Leaflet.js map visualization to this file")
	debugCrossings := flag.Bool("debug-crossings", false, "overlay the fix before and after each cylinder crossing on the HTML map")
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "Usage: %s [flags] <flight.igc>\n\nFlags:\n", os.Args[0])
		flag.PrintDefaults()
	}
	flag.Parse()

	if flag.NArg() < 1 {
		flag.Usage()
		os.Exit(1)
	}
	igcFile := flag.Arg(0)

	// Parse the IGC file.
	flight, err := ParseIGC(igcFile)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error parsing IGC file: %v\n", err)
		os.Exit(1)
	}

	// Optionally load an external waypoints database.
	var wpDB map[string]Waypoint
	if *waypointsFile != "" {
		wpDB, err = ParseWaypointFile(*waypointsFile)
		if err != nil {
			fmt.Fprintf(os.Stderr, "Error parsing waypoints file: %v\n", err)
			os.Exit(1)
		}
		fmt.Printf("Loaded %d waypoints from %s\n", len(wpDB), *waypointsFile)
	}

	// Optionally load an external task file (requires waypoints database).
	var externalTask []Waypoint
	if *taskFile != "" {
		if wpDB == nil {
			fmt.Fprintf(os.Stderr, "Error: --task requires --waypoints\n")
			os.Exit(1)
		}
		externalTask, err = ParseTaskFile(*taskFile, wpDB)
		if err != nil {
			fmt.Fprintf(os.Stderr, "Error parsing task file: %v\n", err)
			os.Exit(1)
		}
	}

	// Print flight summary.
	fmt.Printf("\n=== IGC Flight Summary ===\n")
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

	// Show task: prefer external task file, fall back to IGC-declared task.
	task := flight.Task
	taskSource := "IGC declared"
	if externalTask != nil {
		task = externalTask
		taskSource = *taskFile
	}

	if len(task) > 0 {
		fmt.Printf("\n--- Task (%s): %d waypoints ---\n", taskSource, len(task))
		for i, wp := range task {
			typeStr := wp.Type.String()
			if typeStr == "" {
				typeStr = "enter"
			}
			fmt.Printf("  %d  lat=%9.5f  lon=%10.5f  r=%6gm  %-5s  %s\n",
				i+1, wp.Lat, wp.Lon, wp.Radius, typeStr, wp.Name)
		}

		result := ScoreFlight(flight.Fixes, task, *interpolate)
		splits := result.Splits
		fmt.Printf("\n--- Splits (%d/%d waypoints achieved) ---\n", len(splits), len(task))
		var startTime time.Time
		for i, s := range splits {
			if i == 0 {
				startTime = s.Time
				fmt.Printf("  %d  %-12s  %s  (start)\n",
					i+1, s.Waypoint.Name, s.Time.Format("15:04:05 UTC"))
			} else {
				elapsed := s.Time.Sub(startTime)
				leg := s.Time.Sub(splits[i-1].Time)
				fmt.Printf("  %d  %-12s  %s  elapsed=%s  leg=%s\n",
					i+1, s.Waypoint.Name, s.Time.Format("15:04:05 UTC"),
					formatDuration(elapsed), formatDuration(leg))
			}
		}
		if len(splits) > 0 {
			fmt.Printf("  Speed time: %s\n", formatDuration(result.SpeedTime))
			if result.TaskComplete {
				fmt.Printf("  Task complete.\n")
			} else {
				fmt.Printf("  Task not complete.\n")
			}
		}

		if *htmlFile != "" {
			if err := WriteHTML(*htmlFile, flight, task, splits, *debugCrossings); err != nil {
				fmt.Fprintf(os.Stderr, "Error writing HTML: %v\n", err)
				os.Exit(1)
			}
			fmt.Printf("\nWrote map to %s\n", *htmlFile)
		}
	} else {
		fmt.Printf("\nNo task loaded.\n")

		if *htmlFile != "" {
			if err := WriteHTML(*htmlFile, flight, nil, nil, false); err != nil {
				fmt.Fprintf(os.Stderr, "Error writing HTML: %v\n", err)
				os.Exit(1)
			}
			fmt.Printf("\nWrote map to %s\n", *htmlFile)
		}
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
// Standard IGC task order: Takeoff, Start, TP1...TPn, Finish, Landing.
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
