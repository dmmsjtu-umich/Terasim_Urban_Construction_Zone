# Urban Construction Zone Generator

Generate realistic construction zone scenarios for autonomous vehicle testing in [TeraSim](https://github.com/mcity/TeraSim) simulation platform.

### Construction Zone Visualization

<img src="assets/demo_construction_zone.png" alt="Construction Zone Demo" style="zoom:10%;" />

*Generated construction zone with traffic cones placed via SUMO/TraCI. Colors indicate zone types: orange (taper-in), red (work zone), green (taper-out).*

### TomTom Real-World Data Integration

<img src="assets/demo_tomtom.png" alt="TomTom Demo" style="zoom:10%;" />

*Construction zone generated from real TomTom Traffic API data.*

## Key Features

- **Multi-Mode Generation**: Random, coordinate-based, or TomTom API real-world data
- **MUTCD-Compliant Layout**: Warning → Taper-In → Work Zone → Taper-Out
- **Automatic Edge Selection**: BFS-based path finding for multi-edge zones
- **Multi-Format Visualization**: Static PNG, segmented views, interactive HTML maps

## How It Works

```
                    Construction Zone Layout
    ─────────────────────────────────────────────────────────►
    Vehicle Travel Direction

    |← Warning(optional) →|← Taper-In →|←──── Work Zone ────→|← Taper-Out →|
           🔺 🔺     			🔶 🔶 🔶       🔶 🔶 🔶 🔶 🔶 🔶    🔶 🔶 🔶
```

**Pipeline:**
1. Select road segment (random / coordinates / TomTom API)
2. Calculate zone lengths based on speed limit (MUTCD formulas)
3. Generate YAML config with lane plans
4. Run TeraSim simulation (cones placed via TraCI)
5. Visualize results & export to Waymo format

## Quick Start

```bash
# Random generation
python construction_zone_generator.py random --scene scenes/ann_arbor_whole --num 3

# From coordinates
python construction_zone_generator.py coords --start 42.298,-83.720 --end 42.299,-83.718

# From TomTom API (real-world data)
python construction_zone_generator.py tomtom --api-key YOUR_KEY --center 39.74,-104.99 --size 2000

# Full pipeline (generate → simulate → visualize)
python pipeline.py --scene ann_arbor_whole --num 1 --gui
```

## File Structure

```
├── construction_zone_generator.py  # Main entry point
├── pipeline.py                     # End-to-end simulation pipeline
├── visualizer.py                   # Visualization tools
├── config_generator.py             # Config generation
├── tomtom_pipeline.py              # TomTom API integration
├── configs/                        # YAML templates
├── scenes/                         # Map data (SUMO networks)
└── terasim_nde_nade_adversity/     # Runtime adversity class
```

## Technical Highlights

| Feature | Implementation |
|---------|---------------|
| Coordinate Matching | Lat/Lon → SUMO XY → Nearest edge (100m radius) |
| Zone Length | MUTCD formulas based on speed limit |
| Path Finding | BFS for multi-edge construction zones |
| Object Placement | TraCI stationary vehicles as cones/barriers |
| Visualization | matplotlib + folium (Google satellite tiles) |

## Dependencies

- Python 3.8+
- SUMO 1.23.1+ (`eclipse-sumo`)
- `sumolib`, `matplotlib`, `folium`, `omegaconf`

## Integration

This module is part of the TeraSim ecosystem:
- **terasim-envgen**: Zone geometry generation
- **terasim-nde-nade**: Runtime adversity execution
- **terasim-datazoo**: Waymo format export

## License

Part of [TeraSim](https://github.com/mcity/TeraSim) project.
