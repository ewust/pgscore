package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"os"
	"strconv"
	"strings"
)

// ParseWaypointFile reads an OziExplorer waypoint file (.wpt) and returns a
// map from waypoint name (uppercased) to Waypoint with coordinates.
//
// OziExplorer format:
//
//	OziExplorer Waypoint File Version 1.1
//	WGS 84
//	Reserved 2
//	Reserved 3
//	1,NAME,lat,lon,,0,1,...
func ParseWaypointFile(filename string) (map[string]Waypoint, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	wps := make(map[string]Waypoint)
	scanner := bufio.NewScanner(f)
	lineNum := 0

	for scanner.Scan() {
		lineNum++
		if lineNum <= 4 {
			continue // skip 4-line header
		}
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}

		fields := strings.Split(line, ",")
		if len(fields) < 4 {
			continue
		}

		name := strings.TrimSpace(fields[1])
		lat, err := strconv.ParseFloat(strings.TrimSpace(fields[2]), 64)
		if err != nil {
			return nil, fmt.Errorf("line %d: bad lat %q: %w", lineNum, fields[2], err)
		}
		lon, err := strconv.ParseFloat(strings.TrimSpace(fields[3]), 64)
		if err != nil {
			return nil, fmt.Errorf("line %d: bad lon %q: %w", lineNum, fields[3], err)
		}

		wps[strings.ToUpper(name)] = Waypoint{Lat: lat, Lon: lon, Name: name}
	}

	return wps, scanner.Err()
}

// ParseTaskFile reads a simple task definition file and returns an ordered list
// of Waypoints with coordinates looked up from the provided waypoints map.
//
// Task file format (one waypoint per line after the task name):
//
//	Task Name
//	WPNAME RADIUSm [TYPE]
//
// Examples:
//
//	D14 800M EXIT
//	B6 3500M
//	B25 4000M ES
//	B18 400M GOAL
func ParseTaskFile(filename string, waypoints map[string]Waypoint) ([]Waypoint, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	var task []Waypoint
	scanner := bufio.NewScanner(f)
	lineNum := 0

	for scanner.Scan() {
		lineNum++
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		if lineNum == 1 {
			continue // first non-empty line is the task name
		}

		fields := strings.Fields(line)
		if len(fields) < 2 {
			return nil, fmt.Errorf("line %d: expected \"NAME RADIUSm\", got %q", lineNum, line)
		}

		name := fields[0]
		wp, ok := waypoints[strings.ToUpper(name)]
		if !ok {
			return nil, fmt.Errorf("line %d: waypoint %q not found in waypoints file", lineNum, name)
		}

		// Parse radius: "800M" -> 800.0
		radiusStr := strings.TrimRight(strings.ToUpper(fields[1]), "M")
		radius, err := strconv.ParseFloat(radiusStr, 64)
		if err != nil {
			return nil, fmt.Errorf("line %d: bad radius %q: %w", lineNum, fields[1], err)
		}

		// Parse optional type annotation (3rd field): EXIT, ES, GOAL.
		wpType := WPTypeNormal
		if len(fields) >= 3 {
			switch strings.ToUpper(fields[2]) {
			case "EXIT":
				wpType = WPTypeExit
			case "ES":
				wpType = WPTypeESS
			case "GOAL":
				wpType = WPTypeGoal
			}
		}

		task = append(task, Waypoint{
			Lat:    wp.Lat,
			Lon:    wp.Lon,
			Name:   name,
			Radius: radius,
			Type:   wpType,
		})
	}

	return task, scanner.Err()
}

// xctskFile mirrors the JSON structure of an XCTrack task file (.xctsk).
type xctskFile struct {
	SSS struct {
		Direction string `json:"direction"` // "EXIT" or "ENTER"
	} `json:"sss"`
	Turnpoints []struct {
		Radius   float64 `json:"radius"`
		Waypoint struct {
			Name string  `json:"name"`
			Lat  float64 `json:"lat"`
			Lon  float64 `json:"lon"`
		} `json:"waypoint"`
		Type string `json:"type"` // "SSS", "ESS", or "" for normal/goal
	} `json:"turnpoints"`
}

// ParseXCTaskFile reads an XCTrack task file (.xctsk) and returns an ordered
// list of Waypoints. Coordinates are embedded in the file so no external
// waypoints database is required.
//
// Type mapping:
//
//	"SSS": WPTypeExit  (start: pilot must exit the cylinder)
//	"ESS": WPTypeESS
//	""   : WPTypeGoal on the last turnpoint, WPTypeNormal otherwise
func ParseXCTaskFile(filename string) ([]Waypoint, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, err
	}

	var f xctskFile
	if err := json.Unmarshal(data, &f); err != nil {
		return nil, fmt.Errorf("parsing xctsk JSON: %w", err)
	}

	sssIsExit := f.SSS.Direction == "EXIT"

	task := make([]Waypoint, 0, len(f.Turnpoints))
	for i, tp := range f.Turnpoints {
		var wpType WaypointType
		switch tp.Type {
		case "SSS":
			if sssIsExit {
				wpType = WPTypeExit
			} else {
				wpType = WPTypeNormal
			}
		case "ESS":
			wpType = WPTypeESS
		case "":
			if i == len(f.Turnpoints)-1 {
				wpType = WPTypeGoal
			} else {
				wpType = WPTypeNormal
			}
		}
		task = append(task, Waypoint{
			Lat:    tp.Waypoint.Lat,
			Lon:    tp.Waypoint.Lon,
			Name:   tp.Waypoint.Name,
			Radius: tp.Radius,
			Type:   wpType,
		})
	}

	return task, nil
}
