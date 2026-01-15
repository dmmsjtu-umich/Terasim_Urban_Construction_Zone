# Urban Construction Zone Generator for Autonomous Vehicle Testing

> **Research Project** | University of Michigan - Mcity | TeraSim Simulation Platform

A simulation module that automatically generates realistic highway construction zone scenarios for testing autonomous vehicle decision-making in work zone environments.

## Demo Results

<p align="center">
  <img src="assets/demo_construction_zone.png" width="80%" alt="Construction Zone Visualization"/>
</p>

<p align="center"><em>Generated construction zone with MUTCD-compliant cone placement. Colors: orange (taper-in), red (work zone), green (taper-out)</em></p>

<p align="center">
  <img src="assets/demo_tomtom.png" width="80%" alt="TomTom Real Data"/>
</p>

<p align="center"><em>Construction zone generated from real-world TomTom Traffic API data</em></p>

---

## Motivation

Construction zones are high-risk areas for autonomous vehicles due to:
- Unexpected lane closures and merging patterns
- Non-standard traffic control devices
- Complex interactions with human drivers

This project creates a **scalable simulation pipeline** to generate diverse construction zone scenarios for AV testing, supporting both synthetic and real-world data sources.

---

## Technical Approach

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    		Input Sources                         │
├──────────────────┬──────────────────┬───────────────────────────┤
│  Random Mode     │  Coordinate Mode │  TomTom API Mode          │
│  (Auto-select    │  (User-specified │  (Real-world construction │
│   road segments) │   lat/lon)       │   zone locations)         │
└────────┬─────────┴────────┬─────────┴─────────────┬─────────────┘
         │                  │                       │
         ▼                  ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│              Construction Zone Generator                        │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────────┐    │
│  │ Edge Selection│  │ Zone Layout   │  │ Object Placement  │    │
│  │ (BFS pathfind)│→ │ (MUTCD rules) │→ │ (TraCI interface) │    │
│  └───────────────┘  └───────────────┘  └───────────────────┘    │
└─────────────────────────────┬───────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SUMO/TeraSim Simulation                      │
│         Traffic simulation with construction zone obstacles     │
└─────────────────────────────┬───────────────────────────────────┘
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Output Formats                             │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐     │
│  │ Visualization│  │ FCD Trajectory│ │ Waymo Dataset      │     │
│  │ (PNG/HTML)   │  │ (XML)        │  │ Format (.pb)       │     │
│  └──────────────┘  └──────────────┘  └────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

### MUTCD-Compliant Zone Layout

Construction zones follow U.S. Manual on Uniform Traffic Control Devices (MUTCD) standards:

```
Vehicle Travel Direction ──────────────────────────────────────────►

┌─────────────┬──────────────┬────────────────────┬──────────────┐
│   Warning   │   Taper-In   │     Work Zone      │  Taper-Out   │
│    Zone     │    Zone      │                    │    Zone      │
├─────────────┼──────────────┼────────────────────┼──────────────┤
│  🔺  🔺  🔺 │ 🔶  🔶  🔶  🔶│ 🔶  🔶  🔶  🔶   │  🔶  🔶  🔶  │
│  (signs)    │  (diagonal)  │   (parallel)       │  (diagonal)  │
└─────────────┴──────────────┴────────────────────┴──────────────┘

Zone lengths calculated by MUTCD formulas:
  - Taper Length: L = W × S² / 60  (for S ≤ 40 mph)
  - Taper Length: L = W × S        (for S ≥ 45 mph)
  - Warning Zone: 60m - 450m based on speed limit
```

### Key Algorithms

| Component | Algorithm | Description |
|-----------|-----------|-------------|
| **Edge Selection** | BFS Graph Search | Find connected road segments for extended zones |
| **Coordinate Matching** | KD-Tree + Projection | Convert GPS coordinates to lane positions (100m radius) |
| **Zone Calculation** | MUTCD Formulas | Speed-dependent taper and warning zone lengths |
| **Object Placement** | TraCI API | Place stationary vehicles as cones/barriers in SUMO |

---

## Three Generation Modes

The system supports three data input modes to cover different use cases:

| Mode | Input | Use Case | Example |
|------|-------|----------|---------|
| **Random** | SUMO network + count | Generate diverse synthetic scenarios for batch testing | `--scene ann_arbor --num 100` |
| **Coordinate** | Start/End GPS coords | Test specific real-world locations | `--start 42.28,-83.74 --end 42.29,-83.73` |
| **TomTom API** | API key + search area | Use real active construction zone data | `--center 39.74,-104.99 --size 5000` |

```python
# Example: Three generation modes
generator.random_generate(num_configs=5)           # Mode 1: Synthetic scenarios
generator.generate_from_coordinates(start, end)    # Mode 2: Specific location
generator.generate_from_tomtom(api_key)            # Mode 3: Real-world data
```

---

## Implementation Highlights

### 1. Automatic Lane Compatibility Check
- Filters roads by vehicle accessibility (passenger lanes only)
- Validates minimum lane width (≥2m) and speed limits
- Ensures at least one lane remains open for traffic flow

### 3. Interactive Visualization
- Static PNG renders with matplotlib
- Zoomable HTML maps with Google satellite tiles (zoom level 21)
- Segmented views for long construction corridors (>1km)

---

## Results & Validation

| Metric | Value |
|--------|-------|
| Supported map formats | OpenStreetMap, SUMO networks |
| Generation speed | ~50 scenarios/minute |
| Zone length range | 30m - 10km+ |
| Output formats | PNG, HTML, Waymo .pb, FCD XML |
| Real-world data source | TomTom Traffic API (27+ cities) |

---

## Tech Stack

- **Languages**: Python 3.10
- **Simulation**: SUMO 1.23.1, TraCI API
- **Visualization**: Matplotlib, Folium, Plotly
- **Data Processing**: NumPy, OmegaConf
- **External APIs**: TomTom Traffic API, OpenStreetMap

---

## Usage

```bash
# Generate random construction zones
python construction_zone_generator.py random --scene scenes/ann_arbor --num 5

# Generate from GPS coordinates
python construction_zone_generator.py coords --start 42.28,-83.74 --end 42.29,-83.73

# Fetch real construction zones from TomTom
python construction_zone_generator.py tomtom --api-key YOUR_KEY --center 39.74,-104.99

# Run full simulation pipeline
python pipeline.py --scene ann_arbor_whole --num 1 --gui
```

---

## Project Context

This module was developed as part of the [TeraSim](https://github.com/mcity/TeraSim) project at **University of Michigan Mcity**, an open-source traffic simulation platform for autonomous vehicle testing.

**My Contributions:**
- Designed and implemented the complete construction zone generation pipeline
- Integrated TomTom Traffic API for real-world data acquisition
- Developed MUTCD-compliant zone layout algorithms
- Created multi-format visualization system
- Built Waymo dataset export functionality for ML training

**Skills Demonstrated:**
- Problem Solving: Identified testing gap, designed modular solution
- Visualization: Multi-format outputs (PNG, interactive HTML, satellite maps)
- Software Engineering: Clean Python codebase with API integration
- Domain Knowledge: MUTCD standards, traffic simulation, AV testing

> **For detailed portfolio presentation, see [PORTFOLIO.md](PORTFOLIO.md)**

---

## License

Part of the [TeraSim](https://github.com/mcity/TeraSim) project - University of Michigan.
