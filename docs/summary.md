# Technical Summary

## Overview
This summary highlights the parts of the repo that are most relevant systems thinking, modeling discipline, and engineering communication.

## Mission Definition

The aircraft concept targeted disaster-response operations where ground communications are degraded and situational awareness is limited. The mission centered on a fixed-wing UAV that could:

- travel quickly to the operating area
- remain airborne for extended durations
- carry visual and thermal sensing payloads
- support communications coverage for responders
- operate with short takeoff and landing constraints

The concept was designed around a 120-knot cruise speed, at least 12 hours of endurance, and sub-1000-ft STOL performance. The final design achieved a maximum endurance of 13.5 hours while satisfying the non-tradable customer requirements.

## What This Repository Preserves

### 1. Final design documentation

The main project deliverable is the final report:

- `docs/reports/final-report.pdf`

This document contains the full multidisciplinary design narrative, including subsystem descriptions, interfaces, tradeoffs, and project-level conclusions.

### 2. Flight-dynamics and simulation artifacts

The most technically representative modeling files are grouped under `models/rcam/`:

- `TrimAndLinearize.m` trims and linearizes the aircraft model, then separates longitudinal and lateral dynamics for modal inspection
- `RCAM_Simulation.slx` preserves the main Simulink model
- `references/RCAM_original.m` keeps the reference RCAM implementation for comparison
- `variants/RCAM_model_for_xjh.m` preserves a project-specific RCAM model variant from the original working directory snapshot

These files show a workflow that moves from aircraft dynamics modeling to trim, linearization, and modal interpretation.

### 3. Stability and control analysis artifacts

Two focused analysis areas are preserved under `analysis/`:

- `analysis/stability/hand_calculated_stability.m`
- `analysis/control_surfaces/`

The stability file captures hand-derived mode estimates such as short-period and Dutch-roll behavior. The control-surface scripts preserve hinge-moment and rudder/elevator sizing calculations that complement the broader aircraft model.

### 4. OpenVSP-derived engineering outputs

The final OpenVSP exports are in `models/openvsp/final/`:

- `concept_rev6_mass_properties.txt`
- `concept_rev6_force_coefficients.txt`

These files preserve the final mass-property and force-coefficient data from the configuration snapshot used in the project directory.

Representative values from the preserved mass-properties export include:

- total mass: `24.706390`
- center of gravity: `6.224955, -0.000738, 0.212320`
- principal inertia terms: `224.834198, 227.640065, 429.120628`
