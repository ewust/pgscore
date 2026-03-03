package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"time"
)

func main() {
	taskFile := flag.String("task", "", "task definition file (e.g. inandout.txt)")
	xctaskFile := flag.String("xctask", "", "XCTrack task file (.xctsk JSON)")
	waypointsFile := flag.String("waypoints", "", "OziExplorer waypoints file (.wpt)")
	interpolate := flag.Bool("interpolate", false, "interpolate crossing times to the cylinder boundary (default: use first qualifying fix timestamp)")
	htmlFile := flag.String("html", "", "write a Leaflet.js map visualization to this file")
	debugCrossings := flag.Bool("debug-crossings", false, "overlay the fix before and after each cylinder crossing on the HTML map")
	earthModelFlag := flag.String("earth-model", "wgs84", "earth model for distance calculations: wgs84 (default) or fai (spherical)")
	jsonOut := flag.Bool("json", false, "output results as JSON")
	vizJSON := flag.Bool("viz-json", false, "output visualization JSON (track, waypoints, splits, optimized route) to stdout")
	progressOut := flag.Bool("progress", false, "output a JSON array of [unixTimestamp, distanceMeters] progress tuples to stdout")
	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "Usage: %s [flags] <flight.igc>\n\nFlags:\n", os.Args[0])
		flag.PrintDefaults()
	}
	flag.Parse()

	switch *earthModelFlag {
	case "wgs84":
		earthModel = WGS84Model{}
	case "fai":
		earthModel = FAIModel{}
	default:
		fmt.Fprintf(os.Stderr, "unknown earth model %q: must be wgs84 or fai\n", *earthModelFlag)
		os.Exit(1)
	}

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
	if *taskFile != "" && *xctaskFile != "" {
		fmt.Fprintf(os.Stderr, "Error: --task and --xctask are mutually exclusive\n")
		os.Exit(1)
	}
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
	if *xctaskFile != "" {
		externalTask, err = ParseXCTaskFile(*xctaskFile)
		if err != nil {
			fmt.Fprintf(os.Stderr, "Error parsing xctask file: %v\n", err)
			os.Exit(1)
		}
	}

	// Show task: prefer external task file, fall back to IGC-declared task.
	task := flight.Task
	taskSource := "IGC declared"
	if externalTask != nil {
		task = externalTask
		switch {
		case *xctaskFile != "":
			taskSource = *xctaskFile
		default:
			taskSource = *taskFile
		}
	}

	if len(task) > 0 {
		result := ScoreFlight(flight.Fixes, task, *interpolate)
		splits := result.Splits

		var startTime time.Time
		if len(splits) > 0 {
			startTime = splits[0].Time
		}

		if *progressOut {
			var out [][2]float64
			if len(splits) == 0 {
				out = [][2]float64{{0, 0}}
			} else {
				pts := computeProgress(flight.Fixes, task, splits, result.TotalOptimizedDistance)
				startTime := splits[0].Time
				out = make([][2]float64, len(pts))
				for i, p := range pts {
					out[i] = [2]float64{p.Time.Sub(startTime).Seconds(), p.Progress}
				}
			}
			enc := json.NewEncoder(os.Stdout)
			enc.Encode(out)
		} else if *vizJSON {
			if err := WriteVisualizationJSON(os.Stdout, flight, task, splits); err != nil {
				fmt.Fprintf(os.Stderr, "Error writing viz JSON: %v\n", err)
				os.Exit(1)
			}
		} else if *jsonOut {
			taskName := ""
			if externalTask != nil {
				taskName = taskSource
			}
			writeJSON(flight, taskName, task, result, splits, startTime)
		} else {
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

			fmt.Printf("\n--- Task (%s): %d waypoints ---\n", taskSource, len(task))
			for i, wp := range task {
				typeStr := wp.Type.String()
				if typeStr == "" {
					typeStr = "enter"
				}
				fmt.Printf("  %d  lat=%9.5f  lon=%10.5f  r=%6gm  %-5s  %s\n",
					i+1, wp.Lat, wp.Lon, wp.Radius, typeStr, wp.Name)
			}

			if *debugCrossings {
				startCrossings := findAllCrossings(flight.Fixes, 0, task[0], *interpolate)
				fmt.Printf("\n--- Start crossings (%d) ---\n", len(startCrossings))
				for _, start := range startCrossings {
					fmt.Printf("  start crossing: %v\n", start)
				}
			}

			fmt.Printf("\n--- Splits (%d/%d waypoints achieved) ---\n", len(splits), len(task))
			for i, s := range splits {
				if i == 0 {
					fmt.Printf("  %d  %-12s  %s  (start)\n",
						i+1, s.Waypoint.Name, s.Time.Format("15:04:05 UTC"))
				} else {
					elapsed := s.Time.Sub(startTime)
					leg := s.Time.Sub(splits[i-1].Time)
					speed := splitSpeed(s.Distance, leg)
					fmt.Printf("  %d  %-12s  %s  elapsed=%s  leg=%s  dist=%dm  speed=%.1fkm/h\n",
						i+1, s.Waypoint.Name, s.Time.Format("15:04:05 UTC"),
						formatDuration(elapsed), formatDuration(leg), int(s.Distance), speed)
				}
			}
			if len(splits) > 0 {
				fmt.Printf("  Speed time: %s\n", formatDuration(result.SpeedTime))
				fmt.Printf("  Distance made: %.2f km / %.2f km (optimized)\n",
					result.DistanceMade/1000, result.TotalOptimizedDistance/1000)
				ssd := speedSectionDistance(splits)
				if ssd > 0 {
					fmt.Printf("  Speed section distance: %.2f km\n", float64(ssd)/1000)
					fmt.Printf("  Speed section speed: %.2f km/h\n", splitSpeed(float64(ssd), result.SpeedTime))
				}
				if result.SpeedSectionOptimizedDistance > 0 {
					fmt.Printf("  Speed section optimized distance: %.2f km\n",
						result.SpeedSectionOptimizedDistance/1000)
				}
				if result.TaskComplete {
					fmt.Printf("  Task complete.\n")
				} else {
					fmt.Printf("  Task not complete.\n")
				}
			} else {
				// Print optimized distance for this task
				total, ssd := optimizedTaskDistance(task)
				fmt.Printf("  Optimized distance: %.2f km\n", total/1000)
				fmt.Printf("  Speed section optimized distance: %.2f km\n", ssd/1000)
			}

			if *htmlFile != "" {
				if err := WriteHTML(*htmlFile, flight, task, splits, *debugCrossings); err != nil {
					fmt.Fprintf(os.Stderr, "Error writing HTML: %v\n", err)
					os.Exit(1)
				}
				fmt.Printf("\nWrote map to %s\n", *htmlFile)
			}
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

type splitJSON struct {
	Name     string  `json:"name"`
	Type     string  `json:"type"`
	Time     string  `json:"time"`
	Elapsed  string  `json:"elapsed"`
	Leg      string  `json:"leg,omitempty"`
	Distance int     `json:"distance,omitempty"`
	Speed    float64 `json:"speed_kmh,omitempty"`
}

type outputJSON struct {
	Pilot                         string      `json:"pilot,omitempty"`
	Date                          string      `json:"date"`
	Glider                        string      `json:"glider,omitempty"`
	Task                          string      `json:"task,omitempty"`
	Splits                        []splitJSON `json:"splits"`
	SpeedTime                     string      `json:"speed_time,omitempty"`
	DistanceMade                  int         `json:"distance_made"`
	OptimizedTaskDistance         int         `json:"optimized_task_distance"`
	SpeedSectionDistance          int         `json:"speed_section_distance,omitempty"`
	SpeedSectionOptimizedDistance int         `json:"speed_section_optimized_distance,omitempty"`
	SpeedSectionSpeedKmh          float64     `json:"speed_section_speed_kmh,omitempty"`
	Complete                      bool        `json:"complete"`
}

// splitSpeed returns the average speed in km/h over a split leg, or 0 if
// distance is 0 or the leg duration is non-positive.
func splitSpeed(dist float64, leg time.Duration) float64 {
	if dist <= 0 || leg <= 0 {
		return 0
	}
	return (dist / 1000.0) / leg.Hours()
}

// speedSectionDistance returns the sum of Split.Distance values up to and
// including the ESS waypoint. Returns the full distance if no ESS.
func speedSectionDistance(splits []Split) int {
	var total float64
	for _, s := range splits {
		total += s.Distance
		if s.Waypoint.Type == WPTypeESS {
			return int(total)
		}
	}
	return int(total)
}

func writeJSON(flight *Flight, taskName string, task []Waypoint, result ScoreResult, splits []Split, startTime time.Time) {
	ssd := speedSectionDistance(splits)
	ssdOptimized := result.SpeedSectionOptimizedDistance
	dOptimized := result.TotalOptimizedDistance
	if len(splits) == 0 {
		// No input file given, just process task file
		dOptimized, ssdOptimized = optimizedTaskDistance(task)
	}

	out := outputJSON{
		Pilot:                         flight.PilotName,
		Date:                          flight.Date.Format("2006-01-02"),
		Glider:                        flight.GliderType,
		Task:                          taskName,
		DistanceMade:                  int(result.DistanceMade),
		OptimizedTaskDistance:         int(dOptimized),
		SpeedSectionDistance:          ssd,
		SpeedSectionOptimizedDistance: int(ssdOptimized),
		SpeedSectionSpeedKmh:          splitSpeed(float64(ssd), result.SpeedTime),
		Complete:                      result.TaskComplete,
	}
	if result.SpeedTime > 0 {
		out.SpeedTime = formatDuration(result.SpeedTime)
	}
	for i, s := range splits {
		sj := splitJSON{
			Name:    s.Waypoint.Name,
			Type:    s.Waypoint.Type.String(),
			Time:    s.Time.Format("15:04:05"),
			Elapsed: formatDuration(s.Time.Sub(startTime)),
		}
		if i > 0 {
			leg := s.Time.Sub(splits[i-1].Time)
			sj.Leg = formatDuration(leg)
			sj.Distance = int(s.Distance)
			sj.Speed = splitSpeed(s.Distance, leg)
		}
		out.Splits = append(out.Splits, sj)
	}
	enc := json.NewEncoder(os.Stdout)
	enc.SetIndent("", "  ")
	enc.Encode(out)
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
