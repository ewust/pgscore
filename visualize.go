package main

import (
	"encoding/json"
	"fmt"
	"os"
	"time"
)

// jsLatLon is a lat/lon coordinate for JSON serialisation into Leaflet.
type jsLatLon struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

type jsWaypoint struct {
	Lat    float64 `json:"lat"`
	Lon    float64 `json:"lon"`
	Name   string  `json:"name"`
	Radius float64 `json:"radius"`
	Type   string  `json:"type"` // "EXIT", "ES", "GOAL", or "" for normal
}

type jsSplit struct {
	Lat     float64 `json:"lat"`
	Lon     float64 `json:"lon"`
	Name    string  `json:"name"`
	Time    string  `json:"time"`
	Elapsed string  `json:"elapsed"` // empty for the start split
	Leg     string  `json:"leg"`     // empty for the start split
}

// jsDebugPoint is a fix immediately before or after a cylinder crossing,
// emitted only when debugCrossings is true.
type jsDebugPoint struct {
	Lat   float64 `json:"lat"`
	Lon   float64 `json:"lon"`
	Label string  `json:"label"` // e.g. "before: D14 EXIT" or "after: D14 EXIT"
	Time  string  `json:"time"`
}

// WriteHTML writes a Leaflet.js HTML map to filename showing:
//   - the flight track as a blue polyline
//   - each task waypoint as a labelled circle
//   - each achieved split as a marker with a popup
func WriteHTML(filename string, flight *Flight, task []Waypoint, splits []Split, debugCrossings bool) error {
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()

	// Build track points, skipping invalid fixes to avoid jumps to 0,0.
	track := make([]jsLatLon, 0, len(flight.Fixes))
	for _, fix := range flight.Fixes {
		if fix.Valid {
			track = append(track, jsLatLon{fix.Lat, fix.Lon})
		}
	}

	// Build waypoint data.
	wps := make([]jsWaypoint, 0, len(task))
	for _, wp := range task {
		wps = append(wps, jsWaypoint{
			Lat:    wp.Lat,
			Lon:    wp.Lon,
			Name:   wp.Name,
			Radius: wp.Radius,
			Type:   wp.Type.String(),
		})
	}

	// Build split data, annotating with elapsed/leg times.
	var startTime time.Time
	jsSplits := make([]jsSplit, 0, len(splits))
	for i, s := range splits {
		if i == 0 {
			startTime = s.Time
		}
		var elapsed, leg string
		if i > 0 {
			elapsed = formatDuration(s.Time.Sub(startTime))
			leg = formatDuration(s.Time.Sub(splits[i-1].Time))
		}
		// Use the actual fix position on the track, not the waypoint centre.
		pos := jsLatLon{s.Waypoint.Lat, s.Waypoint.Lon}
		if s.Index < len(flight.Fixes) {
			pos = jsLatLon{flight.Fixes[s.Index].Lat, flight.Fixes[s.Index].Lon}
		}
		jsSplits = append(jsSplits, jsSplit{
			Lat:     pos.Lat,
			Lon:     pos.Lon,
			Name:    s.Waypoint.Name,
			Time:    s.Time.Format("15:04:05 UTC"),
			Elapsed: elapsed,
			Leg:     leg,
		})
	}

	// Optionally collect the fix immediately before and after each crossing.
	var debugPoints []jsDebugPoint
	if debugCrossings {
		for _, s := range splits {
			idx := s.Index
			wpLabel := s.Waypoint.Name
			if s.Waypoint.Type != WPTypeNormal {
				wpLabel += " " + s.Waypoint.Type.String()
			}
			if idx > 0 {
				b := flight.Fixes[idx-1]
				debugPoints = append(debugPoints, jsDebugPoint{
					Lat:   b.Lat,
					Lon:   b.Lon,
					Label: "before: " + wpLabel,
					Time:  b.Timestamp.Format("15:04:05 UTC"),
				})
			}
			if idx < len(flight.Fixes) {
				a := flight.Fixes[idx]
				debugPoints = append(debugPoints, jsDebugPoint{
					Lat:   a.Lat,
					Lon:   a.Lon,
					Label: "after: " + wpLabel,
					Time:  a.Timestamp.Format("15:04:05 UTC"),
				})
			}
		}
	}

	trackJSON, err := json.Marshal(track)
	if err != nil {
		return fmt.Errorf("marshalling track: %w", err)
	}
	wpsJSON, err := json.Marshal(wps)
	if err != nil {
		return fmt.Errorf("marshalling waypoints: %w", err)
	}
	splitsJSON, err := json.Marshal(jsSplits)
	if err != nil {
		return fmt.Errorf("marshalling splits: %w", err)
	}
	debugJSON, err := json.Marshal(debugPoints)
	if err != nil {
		return fmt.Errorf("marshalling debug points: %w", err)
	}

	title := flight.Date.Format("2006-01-02")
	if flight.PilotName != "" {
		title = flight.PilotName + " - " + title
	}

	_, err = fmt.Fprintf(f, leafletHTML, title, title, trackJSON, wpsJSON, splitsJSON, debugJSON)
	return err
}

// leafletHTML is the full HTML page template.
// Placeholders (in order): page title, h1 title, trackJSON, waypointsJSON, splitsJSON.
// Literal % signs in CSS must be written as %%.
const leafletHTML = `<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>%s</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <style>
    html, body { height: 100%%; margin: 0; padding: 0; font-family: sans-serif; }
    #map  { position: absolute; top: 0; bottom: 0; width: 100%%; }
    #info { position: absolute; top: 10px; right: 10px; z-index: 1000;
            background: white; padding: 8px 12px; border-radius: 4px;
            box-shadow: 0 1px 5px rgba(0,0,0,.4); font-size: 13px; max-width: 220px; }
    #info h3 { margin: 0 0 6px; font-size: 14px; }
    .legend-dot { display: inline-block; width: 10px; height: 10px;
                  border-radius: 50%%; margin-right: 5px; }
  </style>
</head>
<body>
<div id="map"></div>
<div id="info">
  <h3>%s</h3>
  <div><span class="legend-dot" style="background:#2980b9"></span>Flight track</div>
  <div><span class="legend-dot" style="background:#27ae60"></span>Start (EXIT)</div>
  <div><span class="legend-dot" style="background:#2980b9"></span>Turnpoint</div>
  <div><span class="legend-dot" style="background:#e67e22"></span>ESS</div>
  <div><span class="legend-dot" style="background:#c0392b"></span>Goal</div>
  <div><span class="legend-dot" style="background:#f39c12"></span>Split</div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
var trackPoints  = %s;
var waypoints    = %s;
var splits       = %s;
var debugPoints  = %s;


var map = L.map('map')
//.setView([trackPoints[0].lat, trackPoints[0].lon], 13);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  maxZoom: 18
}).addTo(map);

// -- Flight track
var trackLayer = null;
if (trackPoints.length > 0) {
  var latlngs = trackPoints.map(function(p) { return [p.lat, p.lon]; });
  trackLayer = L.polyline(latlngs, {color: '#2980b9', weight: 2, opacity: 0.8}).addTo(map);

  L.circleMarker(latlngs[0], {
    radius: 5, color: '#27ae60', fillColor: '#27ae60', fillOpacity: 1, weight: 2
  }).bindTooltip('Track start').addTo(map);

  L.circleMarker(latlngs[latlngs.length - 1], {
    radius: 5, color: '#c0392b', fillColor: '#c0392b', fillOpacity: 1, weight: 2
  }).bindTooltip('Track end').addTo(map);
}

// -- Task cylinders
var typeColors = {'EXIT': '#27ae60', 'ES': '#e67e22', 'GOAL': '#c0392b', '': '#2980b9'};
waypoints.forEach(function(wp) {
  var color = typeColors[wp.type] !== undefined ? typeColors[wp.type] : '#2980b9';
  var label = wp.name + (wp.type ? ' [' + wp.type + ']' : '');

  if (wp.radius > 0) {
    L.circle([wp.lat, wp.lon], {
      radius: wp.radius, color: color, weight: 2, fillOpacity: 0.06
    }).bindTooltip(label).addTo(map);
  }

  L.circleMarker([wp.lat, wp.lon], {
    radius: 4, color: color, fillColor: color, fillOpacity: 1, weight: 2
  }).bindTooltip(label).addTo(map);
});

// -- Split markers
splits.forEach(function(s) {
  var lines = ['<b>' + s.name + '</b>', s.time];
  if (s.elapsed) lines.push('Elapsed: ' + s.elapsed);
  if (s.leg)     lines.push('Leg: '     + s.leg);
  L.circleMarker([s.lat, s.lon], {
    radius: 6, color: '#f39c12', fillColor: '#f39c12', fillOpacity: 1, weight: 2
  }).bindPopup(lines.join('<br>')).addTo(map);
});

// -- Debug crossing points (before/after each split, only when debugCrossings=true)
if (debugPoints) {
  debugPoints.forEach(function(p) {
    var isBefore = p.label.indexOf('before:') === 0;
    var color = isBefore ? '#8e44ad' : '#1abc9c';
    L.circleMarker([p.lat, p.lon], {
      radius: 4, color: color, fillColor: color, fillOpacity: 1, weight: 2
    }).bindTooltip('<b>' + p.label + '</b><br>' + p.time).addTo(map);
  });
}

// -- Fit bounds
if (trackLayer) {
  map.fitBounds(trackLayer.getBounds(), {padding: [20, 20]});
} else if (waypoints.length > 0) {
  map.fitBounds(L.latLngBounds(waypoints.map(function(wp) { return [wp.lat, wp.lon]; })),
                {padding: [40, 40]});
} else {
  map.setView([0, 0], 2);
}
</script>
</body>
</html>
`
