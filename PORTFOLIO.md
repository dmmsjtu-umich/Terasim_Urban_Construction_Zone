# Urban Construction Zone Generator
## Autonomous Vehicle Testing Simulation Module

> **Research Project** | University of Michigan - Mcity
> **Role**: Software Developer & Algorithm Designer
> **Duration**: Fall 2024
> **Tech Stack**: Python, SUMO, TraCI, TomTom API

---

## Problem Statement

Construction zones represent one of the most challenging scenarios for autonomous vehicles:

- **23% of highway fatalities** occur in work zones (NHTSA, 2023)
- Non-standard signage and unexpected lane closures
- Complex merging behaviors with human drivers
- Limited real-world testing data available

**Challenge**: How can we generate diverse, realistic construction zone scenarios at scale to improve AV safety testing?

---

## My Solution

I designed and implemented a **scalable simulation pipeline** that automatically generates MUTCD-compliant construction zones for autonomous vehicle testing.

### System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                      INPUT SOURCES                            │
├────────────────┬─────────────────┬───────────────────────────┤
│  Random Mode   │ Coordinate Mode │    TomTom API Mode        │
│  (Synthetic)   │ (GPS-based)     │    (Real-world data)      │
└───────┬────────┴────────┬────────┴────────────┬──────────────┘
        │                 │                     │
        ▼                 ▼                     ▼
┌──────────────────────────────────────────────────────────────┐
│              CONSTRUCTION ZONE GENERATOR                      │
│  ┌────────────┐   ┌─────────────┐   ┌──────────────────┐     │
│  │   Edge     │   │    Zone     │   │     Object       │     │
│  │ Selection  │ → │   Layout    │ → │   Placement      │     │
│  │   (BFS)    │   │  (MUTCD)    │   │    (TraCI)       │     │
│  └────────────┘   └─────────────┘   └──────────────────┘     │
└──────────────────────────┬───────────────────────────────────┘
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                  SUMO/TeraSim SIMULATION                      │
└──────────────────────────┬───────────────────────────────────┘
                           ▼
┌──────────────────────────────────────────────────────────────┐
│                     OUTPUT FORMATS                            │
│    [PNG/HTML Viz]    [FCD Trajectory]    [Waymo Dataset]     │
└──────────────────────────────────────────────────────────────┘
```

---

## Technical Implementation

### 1. MUTCD-Compliant Zone Layout

I implemented U.S. federal highway standards for work zone design:

```
Vehicle Direction ─────────────────────────────────────────────►

┌───────────┬────────────┬──────────────────┬────────────┐
│  Warning  │  Taper-In  │    Work Zone     │ Taper-Out  │
│   Zone    │   Zone     │                  │   Zone     │
├───────────┼────────────┼──────────────────┼────────────┤
│  🔺 🔺 🔺 │ 🔶 🔶 🔶 🔶 │ 🔶 🔶 🔶 🔶 🔶 🔶 │ 🔶 🔶 🔶 🔶 │
│  (signs)  │ (diagonal) │   (parallel)     │ (diagonal) │
└───────────┴────────────┴──────────────────┴────────────┘

Zone Length Formulas:
  L = W × S² / 60    (S ≤ 40 mph)
  L = W × S          (S ≥ 45 mph)

  Where: W = lane width, S = speed limit
```

### 2. Multi-Source Data Integration

| Generation Mode | Data Source | Use Case |
|-----------------|-------------|----------|
| **Random** | SUMO road network | Batch synthetic testing |
| **Coordinate** | User GPS input | Specific location testing |
| **TomTom API** | Real traffic data | Real-world scenario replication |

### 3. Key Algorithms

| Component | Algorithm | Complexity |
|-----------|-----------|------------|
| Edge Selection | BFS Graph Search | O(V + E) |
| Coordinate Matching | KD-Tree + Projection | O(log n) |
| Zone Calculation | MUTCD Formulas | O(1) |
| Object Placement | TraCI API calls | O(n) |

---

## Visualization Results

### Construction Zone Generation

![Demo 1](assets/demo_construction_zone.png)

*Generated construction zone showing taper-in (orange), work zone (red), and taper-out (green) regions with precise cone placement.*

### Real-World Data Integration

![Demo 2](assets/demo_tomtom.png)

*Construction zone generated from live TomTom Traffic API data - Denver, CO highway work zone.*

### Interactive HTML Map

The system generates zoomable satellite maps for detailed inspection:
- Google satellite tiles (zoom level 21)
- Click-to-inspect cone details
- Multi-layer map options

---

## Quantitative Results

| Metric | Achievement |
|--------|-------------|
| Generation Speed | ~50 scenarios/minute |
| Zone Length Range | 30m - 10km+ |
| Coordinate Accuracy | ±2m (GPS to lane) |
| API Coverage | 27+ major US cities |
| Output Formats | 4 (PNG, HTML, Waymo, FCD) |

---

## Code Sample: Zone Layout Calculation

```python
def calculate_taper_length(lane_width_m: float, speed_mph: float) -> float:
    """
    Calculate MUTCD-compliant taper zone length.

    MUTCD Formula:
      - L = W × S² / 60  for S ≤ 40 mph
      - L = W × S        for S ≥ 45 mph
    """
    W_ft = lane_width_m * 3.28084  # Convert to feet

    if speed_mph <= 40:
        L_ft = W_ft * (speed_mph ** 2) / 60
    elif speed_mph >= 45:
        L_ft = W_ft * speed_mph
    else:
        # Linear interpolation for 40-45 mph
        t = (speed_mph - 40) / 5
        L_low = W_ft * (speed_mph ** 2) / 60
        L_high = W_ft * speed_mph
        L_ft = (1 - t) * L_low + t * L_high

    return L_ft * 0.3048  # Convert back to meters
```

---

## Skills Demonstrated

### Problem Solving
- Identified gap in AV testing infrastructure for work zone scenarios
- Designed modular architecture supporting multiple data sources
- Implemented federal highway standards in simulation environment

### Visualization
- Multi-format output (static PNG, interactive HTML, satellite maps)
- Color-coded zone visualization for quick comprehension
- Segmented views for long construction corridors

### Software Engineering
- Clean, documented Python codebase (~3000 lines)
- API integration (TomTom Traffic, OpenStreetMap)
- Export to industry-standard formats (Waymo Dataset)

### Domain Knowledge
- MUTCD highway safety standards
- Traffic simulation (SUMO/TraCI)
- Autonomous vehicle testing methodologies

---

## Project Links

- **GitHub Repository**: [github.com/dmmsjtu-umich/Terasim_Urban_Construction_Zone](https://github.com/dmmsjtu-umich/Terasim_Urban_Construction_Zone)
- **Parent Project**: [TeraSim - University of Michigan Mcity](https://github.com/mcity/TeraSim)

---

## What I Learned

1. **Systems Thinking**: Designing modular pipelines that connect data sources → processing → simulation → visualization
2. **Standards Compliance**: Translating regulatory documents (MUTCD) into working algorithms
3. **API Integration**: Working with external data sources to enhance simulation realism
4. **Research Context**: Contributing to open-source AV safety research at a major university lab

---

*This project was developed as part of my research work at University of Michigan Mcity, contributing to the TeraSim autonomous vehicle testing platform.*
