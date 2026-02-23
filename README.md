# pgscore

A command-line paragliding flight scorer. Given an IGC flight recorder file and a task definition, `pgscore` determines which task waypoints were achieved, computes split times, and calculates distance made along the optimized route. It can also render an interactive map of the flight and task.

## What it does

- Parses [IGC](https://www.fai.org/page/igc-approved-flight-recorders) flight recorder files (GPS track logs from paragliders and hang gliders)
- Loads task definitions from a simple text file referencing an OziExplorer waypoint database (`.wpt`)
- Scores the flight against the task: determines which cylinders were hit, in order, and records split times
- Tries every possible start crossing to find the best result (most waypoints, then shortest speed time)
- Computes the optimized (shortest) task distance based on cylinder boundaries
- Reports distance made along the optimized route, even for incomplete tasks
- Optionally interpolates crossing times to the exact cylinder boundary rather than using the raw fix timestamp
- Generates an interactive HTML map visualization using Leaflet.js

### Waypoint types

| Type | Keyword | Achievement condition |
|------|---------|----------------------|
| Normal turnpoint | *(none)* | Pilot **enters** the cylinder |
| Start | `EXIT` | Pilot **exits** the cylinder (inside → outside) |
| End of Speed Section | `ES` | Pilot enters the cylinder |
| Goal | `GOAL` | Pilot enters the cylinder |

### Earth models

Two distance models are supported:

- **WGS84** (default) — accurate ellipsoidal geodesy via Vincenty's formulae
- **FAI** — spherical earth using the FAI sphere radius (6371 km), as used in some competition scoring systems

## Building

Requires Go 1.23+.

```
go build -o pgscore .
```

## Usage

```
./pgscore [flags] <flight.igc>
```

### Flags

| Flag | Default | Description |
|------|---------|-------------|
| `-task <file>` | | Task definition file (requires `-waypoints`) |
| `-waypoints <file>` | | OziExplorer waypoints database (`.wpt`) |
| `-html <file>` | | Write a Leaflet.js map to this HTML file |
| `-interpolate` | false | Interpolate split times to the cylinder boundary |
| `-debug-crossings` | false | Show pre/post crossing fixes on the HTML map |
| `-earth-model <model>` | `wgs84` | Distance model: `wgs84` or `fai` |

If no `-task` flag is given, pgscore falls back to any task declared inside the IGC file (C records). This is typically not a task, but rather the flight's turnpoints as recorded by XCTrack/Flyskyhy

## Task file format

The first line is the task name. Each subsequent line defines one waypoint:

```
WPNAME RADIUSm [TYPE]
```

- `WPNAME` — waypoint name as it appears in the `.wpt` file (case-insensitive)
- `RADIUSm` — cylinder radius in metres, e.g. `800M`
- `TYPE` *(optional)* — `EXIT`, `ES`, or `GOAL`

**Example** (`data/flatiron-fiesta.txt`):
```
Flatiron Fiesta
D13 1000M EXIT
B3 5200M
B21 1000M ES
B18 400M GOAL
```

## Examples

Score a flight using a task file and waypoint database, and generate a map:

```sh
./pgscore \
  -task data/flatiron-fiesta.txt \
  -waypoints data/RMHPA-waypoints-2025.wpt \
  -html map.html \
  data/example-flight.igc
```

Score using the FAI spherical earth model:

```sh
./pgscore \
  -task data/flatiron-fiesta.txt \
  -waypoints data/RMHPA-waypoints-2025.wpt \
  -earth-model fai \
  data/example-flight.igc
```

Just view the flight track without a task:

```sh
./pgscore -html track.html data/example-flight.igc
```

## Output

```
=== IGC Flight Summary ===
Date:   2025-07-13
Pilot:  Flatiron Freddy

--- GPS Fixes: 8759 ---
First:    17:53:47 UTC  lat= 40.05625  lon=-105.29980  ...
Last:     20:20:08 UTC  lat= 40.05508  lon=-105.29018  ...
Duration: 2:26:21

--- Task (data/flatiron-fiesta.txt): 4 waypoints ---
  1  lat= 40.04817  lon=-105.29986  r=  1000m  EXIT   D13
  2  lat= 39.91354  lon=-105.29373  r=  5200m  enter  B3
  3  lat= 40.07469  lon=-105.29787  r=  1000m  ES     B21
  4  lat= 40.05553  lon=-105.28963  r=   400m  GOAL   B18

--- Splits (4/4 waypoints achieved) ---
  1  D13           18:10:13 UTC  (start)
  2  B3            19:18:11 UTC  elapsed=1:07:58  leg=1:07:58
  3  B21           20:05:37 UTC  elapsed=1:55:24  leg=0:47:26
  4  B18           20:07:58 UTC  elapsed=1:57:45  leg=0:02:21
  Speed time: 1:55:24
  Distance made: 21.33 km / 21.33 km (optimized)
  Task complete.
```
