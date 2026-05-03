# CENG4120-CTS

Python implementation for the CENG4120 Clock Tree Synthesis (CTS) project.

## Overview

This solver builds routing trees from taps to pins on a 2D grid with:
- edge capacity constraints,
- rectangular blockages,
- tap load limits,
- and a runtime budget.

The implementation improves legality and overall evaluator score by combining:
- regret-based, load-aware pin assignment,
- congestion-aware A* routing,
- Steiner-like incremental tree construction,
- and iterative rip-up / reroute refinement.

## Features

- Fast blocked-edge precomputation.
- Regret-based pin-to-tap assignment with load balancing.
- A* routing with congestion and history penalties.
- Per-tap tree growth using reusable routed edges (Steiner-tree-like).
- Iterative rerouting of high-congestion trees before timeout.
- Best-solution snapshot preserved throughout refinement.
- Compatible with course evaluator formats.

## Algorithm

### 1. Input parsing
The program parses:
- `MAXRUNTIME` / `MAX_RUNTIME` (defaults to 300 seconds if absent)
- `MAXLOAD` / `MAX_LOAD`
- `GRIDSIZE` / `GRID_SIZE`
- `CAPACITY`
- `PINS`
- `TAPS`
- `BLKS`

> Note: If `GRIDSIZE`, `MAXLOAD`, or `CAPACITY` are missing, the solver exits silently without producing output.

### 2. Blocked-edge precomputation
All blocked grid edges are converted into a hash set at startup to minimize routing overhead.

### 3. Pin assignment
Pins are assigned to taps using a regret-based heuristic:
- Compute Manhattan distances from each pin to all taps.
- Prioritise pins with high regret (large difference between best and second-best tap distances).
- Include load balancing to avoid exceeding `MAXLOAD`.

### 4. Routing
For each tap cluster (processed largest-first), a routing tree is built incrementally:
- Start from the tap.
- Repeatedly connect the nearest unconnected pin to the current tree using A*.
- A* costs are based on wirelength, edge congestion (relative to capacity), and accumulated history penalties.
- Existing edges already in the tap's tree are reused at near-zero cost (Steiner-tree-like behaviour).

### 5. Iterative Refinement
After the initial solution (up to 15 rounds, subject to runtime deadline):
- Identify congested edges (usage > capacity).
- Increase history costs for those edges.
- Rip-up and reroute the top ~20% most congested tap trees.
- Keep the best legal solution found across all rounds.

## Usage

```bash
python3 cts.py --input test.in --output out.txt
```

## Authors
Law Sau Ho
