# CENG4120-CTS

Python implementation for the CENG4120 Clock Tree Synthesis (CTS) project.

## Overview

This solver builds routing trees from taps to pins on a 2D grid with:
- edge capacity constraints,
- rectangular blockages,
- tap load limits,
- and a runtime budget.

The implementation is designed to improve legality and overall evaluator score by combining:
- capacity-aware pin assignment,
- congestion-aware A* routing,
- Steiner-like incremental tree construction,
- and iterative rip-up / reroute refinement.

## Features

- Fast blocked-edge precomputation.
- Regret-based pin-to-tap assignment with light balancing.
- A* routing with congestion and history penalties.
- Per-tap tree growth using reusable routed edges (Steiner-tree-like).
- Iterative rerouting of high-congestion trees before timeout.
- Compatible with course evaluator formats.

## Algorithm

### 1. Input parsing
The program parses:
- `MAXRUNTIME` / `MAX_RUNTIME`
- `MAXLOAD` / `MAX_LOAD`
- `GRIDSIZE` / `GRID_SIZE`
- `CAPACITY`
- `PINS`
- `TAPS`
- `BLKS`

### 2. Blocked-edge precomputation
All blocked grid edges are converted into a hash set at startup to minimize routing overhead.

### 3. Pin assignment
Pins are assigned to taps using a regret-based heuristic:
- Compute distances from each pin to all taps.
- Prioritize pins with high regret (large difference between best and second-best taps).
- Include load-balancing to avoid exceeding `MAXLOAD`.
- Local search pass to refine assignments.

### 4. Routing
For each tap cluster, a routing tree is built incrementally:
- Start from the tap.
- Repeatedly connect the nearest unconnected pin to the current tree.
- Routing uses A* with costs based on wirelength, congestion, and history.

### 5. Iterative Refinement
After the initial solution:
- Identify congested edges.
- Increase history costs for those edges.
- Rip-up and reroute the most congested trees.
- Keep the best legal solution found.

## Usage

```bash
python3 cts.py --input test.in --output out.txt
```

## Authors
Law Sau Ho
