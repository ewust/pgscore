// Package pgscore provides types and parsing for IGC flight recorder files.
package main

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"
)

// Fix represents a single GPS fix from a B record in an IGC file.
type Fix struct {
	Timestamp   time.Time
	Lat         float64 // decimal degrees, positive=North, negative=South
	Lon         float64 // decimal degrees, positive=East, negative=West
	PressureAlt int     // barometric altitude in meters (ICAO ISA)
	GNSSAlt     int     // GPS altitude in meters (WGS-84 ellipsoid)
	Valid       bool    // true if fix validity is 'A' (3D fix), false for 'V'
}

// WaypointType describes the role a waypoint plays in scoring.
type WaypointType int

const (
	WPTypeNormal WaypointType = iota // standard entry turnpoint
	WPTypeExit                       // start: pilot must EXIT the cylinder to start the clock
	WPTypeESS                        // end of speed section (last timed gate)
	WPTypeGoal                       // goal/finish cylinder
)

func (t WaypointType) String() string {
	switch t {
	case WPTypeExit:
		return "EXIT"
	case WPTypeESS:
		return "ES"
	case WPTypeGoal:
		return "GOAL"
	default:
		return ""
	}
}

// Waypoint represents a task waypoint, typically from a C record or external file.
type Waypoint struct {
	Lat    float64
	Lon    float64
	Name   string       // waypoint identifier
	Radius float64      // cylinder radius in meters (0 = not specified)
	Type   WaypointType // role in scoring (normal entry, exit-start, ESS, goal)
}

// Flight holds all data parsed from an IGC file.
type Flight struct {
	Date       time.Time  // UTC date of the flight
	PilotName  string     // from HFPLT header
	GliderType string     // from HFGTY header
	GliderID   string     // from HFGID header
	Fixes      []Fix      // chronological GPS fixes from B records
	Task       []Waypoint // declared task from C records (takeoff/start/TPs/finish(ess?)/landing)
}

// Note: some instruments (flyskyhigh?) put the task points/reached events inline with waypoints
// others seem not to (xctrack)

// parseLat parses an 8-character IGC latitude string "DDMMmmmN" or "DDMMmmmS"
// into decimal degrees (negative for South).
func parseLat(s string) (float64, error) {
	if len(s) < 8 {
		return 0, fmt.Errorf("lat too short: %q", s)
	}
	deg, err := strconv.Atoi(s[0:2])
	if err != nil {
		return 0, fmt.Errorf("lat degrees %q: %w", s[0:2], err)
	}
	// MMmmm as a single integer: whole minutes * 1000 + decimal minutes
	minInt, err := strconv.Atoi(s[2:7])
	if err != nil {
		return 0, fmt.Errorf("lat minutes %q: %w", s[2:7], err)
	}
	lat := float64(deg) + float64(minInt)/60000.0
	if s[7] == 'S' {
		lat = -lat
	}
	return lat, nil
}

// parseLon parses a 9-character IGC longitude string "DDDMMmmmE" or "DDDMMmmmW"
// into decimal degrees (negative for West).
func parseLon(s string) (float64, error) {
	if len(s) < 9 {
		return 0, fmt.Errorf("lon too short: %q", s)
	}
	deg, err := strconv.Atoi(s[0:3])
	if err != nil {
		return 0, fmt.Errorf("lon degrees %q: %w", s[0:3], err)
	}
	// MMmmm as a single integer: whole minutes * 1000 + decimal minutes
	minInt, err := strconv.Atoi(s[3:8])
	if err != nil {
		return 0, fmt.Errorf("lon minutes %q: %w", s[3:8], err)
	}
	lon := float64(deg) + float64(minInt)/60000.0
	if s[8] == 'W' {
		lon = -lon
	}
	return lon, nil
}

// ParseIGC reads an IGC file and returns a Flight with all parsed data.
// Malformed individual records are skipped; only file-level errors are returned.
func ParseIGC(filename string) (*Flight, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	flight := &Flight{}
	scanner := bufio.NewScanner(f)
	var flightDate time.Time

	for scanner.Scan() {
		line := scanner.Text()
		if len(line) == 0 {
			continue
		}

		switch line[0] {
		case 'H':
			parseHRecord(line, flight, &flightDate)
		case 'B':
			fix, err := parseBRecord(line, flightDate, flight.Fixes)
			if err != nil {
				continue // skip malformed fixes
			}
			flight.Fixes = append(flight.Fixes, fix)
		case 'C':
			wp, ok := parseCRecord(line)
			if ok {
				flight.Task = append(flight.Task, wp)
			}
		}
	}

	return flight, scanner.Err()
}

// parseHRecord extracts pilot, glider, and date info from an H (header) record.
func parseHRecord(line string, flight *Flight, date *time.Time) {
	upper := strings.ToUpper(line)

	// Date record: HFDTE or HFDTE followed by DATE:DDMMYY or directly DDMMYY.
	// Examples:
	//   HFDTE190925
	//   HFDTEDATE:021025,01
	if strings.HasPrefix(upper, "HFDTE") {
		s := line[5:]
		if strings.HasPrefix(strings.ToUpper(s), "DATE:") {
			s = s[5:]
		}
		// s should now start with DDMMYY
		if len(s) >= 6 {
			day, _ := strconv.Atoi(s[0:2])
			month, _ := strconv.Atoi(s[2:4])
			year, _ := strconv.Atoi(s[4:6])
			if year < 70 {
				year += 2000
			} else {
				year += 1900
			}
			*date = time.Date(year, time.Month(month), day, 0, 0, 0, 0, time.UTC)
			flight.Date = *date
		}
		return
	}

	// Pilot: HFPLTPILOTINCHARGE:Name or HFPLTPILOT:Name or HPPLTPILOT:Name etc.
	if strings.HasPrefix(upper, "HFPLT") || strings.HasPrefix(upper, "HPPLT") {
		if idx := strings.Index(line, ":"); idx >= 0 {
			flight.PilotName = strings.TrimSpace(line[idx+1:])
		}
		return
	}

	// Glider type: HFGTYGLIDERTYPE:Type
	if strings.HasPrefix(upper, "HFGTY") || strings.HasPrefix(upper, "HPGTY") {
		if idx := strings.Index(line, ":"); idx >= 0 {
			flight.GliderType = strings.TrimSpace(line[idx+1:])
		}
		return
	}

	// Glider ID / registration: HFGIDGLIDERID:ID
	if strings.HasPrefix(upper, "HFGID") || strings.HasPrefix(upper, "HPGID") {
		if idx := strings.Index(line, ":"); idx >= 0 {
			flight.GliderID = strings.TrimSpace(line[idx+1:])
		}
		return
	}
}

// parseBRecord parses a B (fix) record into a Fix.
// prevFixes is used to detect and correct midnight UTC rollovers.
//
// B record layout (0-indexed):
//
//	B HHMMSS DDMMmmmN DDDMMmmmE [AV] PPPPP GGGGG [extensions...]
//	0 123456 78901234 567890123  4   56789 01234
//	             1111 111111222  2   22222 33333
func parseBRecord(line string, flightDate time.Time, prevFixes []Fix) (Fix, error) {
	if len(line) < 35 {
		return Fix{}, fmt.Errorf("B record too short (%d chars)", len(line))
	}

	hour, e1 := strconv.Atoi(line[1:3])
	min, e2 := strconv.Atoi(line[3:5])
	sec, e3 := strconv.Atoi(line[5:7])
	if e1 != nil || e2 != nil || e3 != nil {
		return Fix{}, fmt.Errorf("invalid time in B record: %q", line[1:7])
	}

	lat, err := parseLat(line[7:15])
	if err != nil {
		return Fix{}, err
	}
	lon, err := parseLon(line[15:24])
	if err != nil {
		return Fix{}, err
	}

	valid := line[24] == 'A'
	pressureAlt, _ := strconv.Atoi(line[25:30])
	gnssAlt, _ := strconv.Atoi(line[30:35])

	ts := time.Date(flightDate.Year(), flightDate.Month(), flightDate.Day(),
		hour, min, sec, 0, time.UTC)

	// Correct for midnight UTC rollover (rare but possible for overnight flights).
	if len(prevFixes) > 0 {
		prev := prevFixes[len(prevFixes)-1].Timestamp
		if ts.Before(prev) {
			ts = ts.Add(24 * time.Hour)
		}
	}

	return Fix{
		Timestamp:   ts,
		Lat:         lat,
		Lon:         lon,
		PressureAlt: pressureAlt,
		GNSSAlt:     gnssAlt,
		Valid:       valid,
	}, nil
}

// parseCRecord parses a C (task declaration) record into a Waypoint.
// Returns (wp, true) for waypoint records, (zero, false) for task-header records.
//
// Waypoint C records have lat/lon at bytes 1–17:
//
//	C DDMMmmmN DDDMMmmmE Description
//	0 12345678 901234567 8+
//
// Task-header C records (first two C records) start with a date/time digit sequence,
// so byte 8 is a digit rather than 'N' or 'S'. We use that to distinguish them.
func parseCRecord(line string) (Waypoint, bool) {
	if len(line) < 18 {
		return Waypoint{}, false
	}

	// Byte 8 is the lat hemisphere indicator (N/S) for waypoint records.
	if line[8] != 'N' && line[8] != 'S' {
		return Waypoint{}, false
	}

	lat, err := parseLat(line[1:9])
	if err != nil {
		return Waypoint{}, false
	}
	lon, err := parseLon(line[9:18])
	if err != nil {
		return Waypoint{}, false
	}

	// Skip null waypoints – some loggers emit 0N 0E for unset takeoff/landing.
	if lat == 0 && lon == 0 {
		return Waypoint{}, false
	}

	name := ""
	if len(line) > 18 {
		name = strings.TrimSpace(line[18:])
	}

	return Waypoint{Lat: lat, Lon: lon, Name: name}, true
}
