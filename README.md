# Urban Construction Zone Module

A comprehensive module for generating realistic urban construction zone scenarios in TeraSim traffic simulations. This module enables automated creation of construction zone environments for testing autonomous vehicle behavior in work zone scenarios.

## Contributions & Features Implemented

This module implements the following key features:

### Core Capabilities
- **Multi-Mode Construction Zone Generation**: Support for random, coordinate-based, and TomTom API-based generation
- **MUTCD-Compliant Zone Layout**: Warning zone, taper-in, work zone, and taper-out following US standards
- **Intelligent Edge Selection**: BFS-based multi-edge path finding for extended construction zones
- **Complete Simulation Pipeline**: End-to-end workflow from config generation to Waymo format export

### Key Implementation Highlights
1. **Automatic Lane Compatibility Checking**: Filters roads by passenger vehicle accessibility, lane width, and speed limits
2. **Dynamic Spacing Calculation**: Speed-dependent object spacing following MUTCD guidelines
3. **Multi-Format Visualization**: Static PNG, segmented images for long zones, and interactive HTML maps with satellite imagery
4. **Real-World Data Integration**: TomTom Traffic API support for fetching actual construction zone locations
5. **Waymo Dataset Conversion**: Export simulation results to Waymo Open Dataset format for ML training

### Technical Achievements
- Coordinate-to-lane projection algorithm with 100m matching radius
- BFS-based path finding for multi-edge construction zones
- MUTCD formula implementation for taper length calculation
- Google satellite tile integration for high-zoom interactive maps (zoom level 21)

## Overview

This module provides three methods to generate construction zones:

| Mode | Function | Input | Existing Scene Required |
|------|----------|-------|-------------------------|
| `random` | Random generation | Scene + count | Yes (or provide center coords to auto-create) |
| `coords` | Coordinate-based | Start/end lat/lon | Optional (auto-creates if not provided) |
| `tomtom` | TomTom API | API key + center + range | Optional (auto-creates if not provided) |

## File Description

| File | Description |
|------|-------------|
| `construction_zone_generator.py` | **Main entry point** - Supports random, coordinate, and TomTom generation |
| `pipeline.py` | Simulation pipeline (generate config → run simulation → visualize → Waymo conversion) |
| `visualizer.py` | Visualization tools (PNG static images + HTML interactive maps) |
| `config_generator.py` | Config generator (low-level utility) |
| `tomtom_pipeline.py` | TomTom API standalone pipeline |

## Quick Start

```bash
cd /home/dmmsjtu/Desktop/TeraSim-main/packages/terasim-envgen/terasim_envgen/core/urban_construcion_zone
source ~/miniconda3/etc/profile.d/conda.sh && conda activate terasim-agent
```

### Mode 1: Random Generation (random)

Randomly select roads in an existing scene to generate construction zones.

```bash
# Use existing scene
python construction_zone_generator.py random \
    --scene scenes/ann_arbor_whole \
    --num 5

# Auto-create new scene (provide center coordinates)
python construction_zone_generator.py random \
    --center 42.28,-83.74 \
    --size 500 \
    --num 3

# Run simulation immediately after generation
python construction_zone_generator.py random \
    --scene scenes/ann_arbor_whole \
    --num 1 \
    --run
```

### Mode 2: Coordinate-Based Generation (coords)

Generate construction zones from specified start/end lat/lon coordinates.

```bash
# Use existing scene
python construction_zone_generator.py coords \
    --scene scenes/ann_arbor_whole \
    --start 42.280,-83.738 \
    --end 42.281,-83.736

# Auto-create new scene (no --scene needed)
python construction_zone_generator.py coords \
    --start 42.29799971530919,-83.72030298642086 \
    --end 42.29818025072757,-83.71945138489987

# Specify scene size (optional)
python construction_zone_generator.py coords \
    --start 42.298,-83.720 \
    --end 42.299,-83.718 \
    --size 800

# Custom zone ID + run simulation
python construction_zone_generator.py coords \
    --start 42.298,-83.720 \
    --end 42.299,-83.718 \
    --id my_zone \
    --run
```

**Auto Scene Creation Notes**:
- When `--scene` is not provided, the center point is auto-calculated from start/end coordinates
- Scene size defaults to 1.5x the coordinate distance, minimum 500 meters
- Use `--size` to manually specify scene size

### Mode 3: TomTom API (tomtom)

Fetch real construction zone data from TomTom Traffic API.

```bash
# Use existing scene (bbox extracted from scene)
python construction_zone_generator.py tomtom \
    --api-key YOUR_TOMTOM_API_KEY \
    --scene scenes/ann_arbor_whole

# Auto-create new scene (provide center coordinates)
python construction_zone_generator.py tomtom \
    --api-key YOUR_TOMTOM_API_KEY \
    --center 42.28,-83.74 \
    --size 1000

# Use API key from environment variable
export TOMTOM_API_KEY=your_key_here
python construction_zone_generator.py tomtom \
    --scene scenes/ann_arbor_whole

# Test with sample data (no API key needed)
python tomtom_pipeline.py --sample --config-only
```

## Visualization

```bash
# Generate PNG static image + HTML interactive map
python visualizer.py \
    --net scenes/ann_arbor_whole/map.net.xml \
    --fcd outputs/zone_xxx/fcd_all.xml \
    --output outputs/zone_xxx/construction_zone.png

# Generate interactive HTML map only
python visualizer.py \
    --net scenes/ann_arbor_whole/map.net.xml \
    --fcd outputs/zone_xxx/fcd_all.xml \
    --output outputs/zone_xxx/map.html \
    --mode interactive

# Long-distance construction zone segmented visualization
python visualizer.py \
    --net scenes/scene_xxx/map.net.xml \
    --fcd outputs/zone_xxx/fcd_all.xml \
    --output outputs/zone_xxx/construction_zone.png \
    --mode segmented \
    --segment-length 500
```

**HTML Map Features**:
- Multiple base map options: Google Satellite, Street Map, Dark Mode, etc.
- Supports high zoom levels (zoom 21)
- Cones colored by zone: orange=taper_in, red=work, green=taper_out
- Click cones to show detailed information

## Using pipeline.py for Full Workflow

```bash
# Basic usage
python pipeline.py --scene ann_arbor_whole --num 1

# With GUI
python pipeline.py --scene ann_arbor_whole --gui

# Full parameters
python pipeline.py \
    --scene ann_arbor_whole \
    --num 5 \
    --width 2.0 \
    --type cone \
    --warning \
    --seed 42
```

## Python API

```python
from construction_zone_generator import ConstructionZoneGenerator

# ===== Method 1: Use existing scene =====
gen = ConstructionZoneGenerator(0, 0)  # placeholder coords
gen.use_existing_scene("scenes/ann_arbor_whole")

# ===== Method 2: Create new scene =====
gen = ConstructionZoneGenerator(
    center_lat=42.28,
    center_lon=-83.74,
    size_meters=500,
)
gen.setup_scene()  # Download OSM and generate SUMO map

# ===== Random generation =====
configs = gen.random_generate(
    num_configs=5,
    min_length=30,
    max_length=100,
    run_simulation=False,
)

# ===== Generate from coordinates =====
config = gen.generate_from_coordinates(
    start_lat=42.280,
    start_lon=-83.738,
    end_lat=42.281,
    end_lon=-83.736,
    zone_id="custom_zone",
    run_simulation=False,
)

# ===== Fetch from TomTom =====
configs = gen.generate_from_tomtom(
    api_key="YOUR_API_KEY",
    run_simulation=False,
)
```

## Coordinate Positioning Algorithm

When start/end lat/lon coordinates are provided, the system:

1. **Lat/Lon to XY**: `convertLonLat2XY()` converts WGS84 coordinates to SUMO network coordinates
2. **Match Edge**: `getNeighboringEdges()` searches for nearest road within 100m radius
3. **Project to Lane**: Projects point onto lane centerline, calculates precise meter position
4. **Path Finding**: If start and end are on different edges, uses BFS to find connecting path

```
User coordinates (42.298, -83.720)
        ↓ convertLonLat2XY()
SUMO XY (1234.5, 5678.9)
        ↓ getNeighboringEdges()
Nearest Edge: "-441142720"
        ↓ _get_position_on_lane()
Projected position: 45.3m
        ↓
Construction zone start = 45.3m
```

## Construction Zone Layout

```
Vehicle travel direction →

|←─ warning ─→|←─ taper_in ─→|←───── work ─────→|←─ taper_out ─→|
  Warning Zone    Taper In         Work Zone         Taper Out

    🔺  🔺  🔺     🔶 🔶 🔶 🔶     🔶 🔶 🔶 🔶 🔶 🔶     🔶 🔶 🔶 🔶
  (Warning signs) (Diagonal cones)  (Straight cones)   (Diagonal cones)
```

## Directory Structure

```
urban_construcion_zone/
├── construction_zone_generator.py  # Main entry point
├── pipeline.py                     # Simulation pipeline
├── visualizer.py                   # Visualization tools
├── config_generator.py             # Config generator
├── tomtom_pipeline.py              # TomTom pipeline
├── loader.py                       # Config loader utility
├── README.md                       # This document
├── configs/                        # YAML templates
│   ├── base.yaml
│   ├── environment.yaml
│   └── adversities/
├── scenes/                         # Scene directory
│   ├── ann_arbor_whole/           # Ann Arbor preset scene
│   │   ├── map.net.xml
│   │   ├── map.osm
│   │   └── simulation_no_traffic.sumocfg
│   └── scene_42.2981_-83.7199_500/ # Auto-created scene
└── outputs/                        # Output directory
    └── zone_xxx_12345/
        ├── fcd_all.xml            # Trajectory data
        ├── construction_zone.png  # Static visualization
        ├── construction_zone.html # Interactive map
        └── waymo_scenario.pb      # Waymo format
```

## CLI Arguments Reference

### construction_zone_generator.py

```bash
# ============ random mode ============
python construction_zone_generator.py random [OPTIONS]

Required (one of):
  --scene PATH      Use existing scene directory
  --center LAT,LON  Center coordinates (auto-create scene)

Optional:
  --size METERS     Map range (meters, default 500)
  --num N           Number of configs to generate (default 5)
  --min-length M    Minimum zone length (default 30m)
  --max-length M    Maximum zone length (default 100m)
  --seed N          Random seed
  --run             Run simulations after generation

# ============ coords mode ============
python construction_zone_generator.py coords [OPTIONS]

Required:
  --start LAT,LON   Start coordinates
  --end LAT,LON     End coordinates

Optional:
  --scene PATH      Use existing scene (auto-creates if not provided)
  --size METERS     Scene size (for auto-creation, default auto-calculated)
  --id NAME         Zone ID (default "manual")
  --run             Run simulation after generation

# ============ tomtom mode ============
python construction_zone_generator.py tomtom [OPTIONS]

Required (API key source, one of):
  --api-key KEY     TomTom API key
  env variable      TOMTOM_API_KEY

Required (scene source, one of):
  --scene PATH      Use existing scene
  --center LAT,LON  Center coordinates (auto-create scene)

Optional:
  --size METERS     Search range (default 1000m)
  --run             Run simulations after generation
```

### pipeline.py

```bash
python pipeline.py [OPTIONS]

  --scene NAME      Scene name (default ann_arbor_whole)
  --num N           Number of configs to generate (default 1)
  --width W         Work zone width (default 2.0m)
  --type TYPE       cone|barrier|mixed (default cone)
  --warning         Enable warning zone
  --gui             Enable SUMO GUI
  --seed N          Random seed
  --skip-vis        Skip visualization
  --skip-waymo      Skip Waymo conversion
```

### visualizer.py

```bash
python visualizer.py [OPTIONS]

Required:
  --net PATH        SUMO network file (map.net.xml)
  --fcd PATH        FCD trajectory file (fcd_all.xml)

Optional:
  --output PATH     Output file path (default auto-generated)
  --mode MODE       static|interactive|segmented (default auto)
  --segment-length  Segment length (segmented mode, default 500m)
  --time TIME       Time point to extract cones (default 5.0s)
```

## TomTom API

### Getting API Key

1. Visit [TomTom Developer Portal](https://developer.tomtom.com/)
2. Register account and create application
3. Get API Key

### Reference: Construction Zones by City

```
Los Angeles: ~35 RoadWorks
Denver: ~27 RoadWorks
New York: ~20 RoadWorks
Chicago: ~15 RoadWorks
Ann Arbor: Usually fewer
```

## Constraints

| Constraint | Value | Description |
|------------|-------|-------------|
| Minimum zone length | 20m | Ensure enough space for cone placement |
| Boundary margin | 5m | Safety distance from lane boundaries |
| Minimum lane count | 2 | Need at least one passable lane |
| Coordinate matching distance | 100m | Maximum matching distance from lat/lon to road |
| Cone spacing | 2m | Default spacing between cones in work zone |

## Dependencies

```bash
# Core dependencies
pip install sumolib pyyaml matplotlib folium

# Optional dependencies
pip install osmnx    # Auto-create scenes (download from OSM)
pip install requests # TomTom API calls
```

- SUMO (eclipse-sumo) 1.23.1+
- Python 3.8+

## FAQ

### Q: No eligible edges found?

A: No multi-lane roads in the scene (need at least 2 lanes), or road length insufficient (<20m).

### Q: Coordinate matching failed?

A: Coordinates are more than 100m from nearest road. Ensure coordinates are within scene bounds.

### Q: TomTom returns empty results?

A: No construction zones in the area, or API key issue. Try other cities like Denver or Los Angeles.

### Q: Cones not showing in simulation?

A: Check for `Skip {edge}: lane_count < 2` warnings in logs, indicating insufficient lane count.

### Q: HTML map satellite view can't zoom in?

A: Fixed. Now uses Google satellite tile source, supports zoom level 21.

### Q: Long-distance construction zone visualization incomplete?

A: Use segmented visualization mode:
```bash
python visualizer.py --mode segmented --segment-length 500
```

## Module Architecture

```
urban_construcion_zone/
├── construction_zone_generator.py  # Main entry - zone layout calculation & generation
├── config_generator.py             # Simulation config generation with adversity injection
├── pipeline.py                     # End-to-end simulation pipeline orchestration
├── visualizer.py                   # Multi-mode visualization (static/segmented/interactive)
├── tomtom_pipeline.py              # TomTom Traffic API integration
├── loader.py                       # Configuration file loader utility
├── README.md                       # This documentation
├── SUMMARY.md                      # Detailed module summary
├── configs/                        # YAML configuration templates
│   ├── base/                       # Base simulation configs
│   └── adversities/static/         # Construction zone adversity configs
├── scenes/                         # Map data for different locations
└── outputs/                        # Simulation output directory
```

## Integration with TeraSim Ecosystem

This module integrates with multiple TeraSim packages:

| Package | Integration |
|---------|-------------|
| **terasim-envgen** | Configuration and zone geometry generation |
| **terasim-nde-nade** | Runtime adversity execution via `SimpleUrbanConstructionAdversity` |
| **terasim-vis** | Network visualization utilities |
| **terasim-datazoo** | Waymo Open Dataset format conversion |
| **terasim-service** | Config path resolution for simulation |

## License

Part of the TeraSim project. See the main repository for license information.
