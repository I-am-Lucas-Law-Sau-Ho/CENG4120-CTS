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
- and limited rip-up / reroute refinement.

## Features

- Fast blocked-edge precomputation.
- Regret-based pin-to-tap assignment with light balancing.
- A* routing with congestion and history penalties.
- Per-tap tree growth using reusable routed edges.
- Iterative rerouting of weaker trees before timeout.
- Same input/output format as the course evaluator.

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
Instead of checking every blockage during every A* expansion, the solver converts all blocked grid edges into a hash set once at startup. This significantly reduces routing overhead.

### 3. Pin assignment
Pins are assigned to taps using a regret-based heuristic:
- compute distances from each pin to all taps,
- prioritize pins whose second-best tap is much worse than their best tap,
- include a load-balance penalty to avoid poor clustering.

A small local improvement pass then moves pins between taps when it reduces estimated assignment cost.

### 4. Routing
For each tap cluster, the solver incrementally builds a routing tree:
- start from the tap,
- repeatedly connect one unconnected pin to the existing tree,
- allow connection to any already-routed tree node.

Routing uses A* with a composite edge cost:
- base wirelength cost,
- congestion penalty from current global edge usage,
- history penalty for edges that repeatedly cause routing difficulty,
- low reuse cost for edges already used by the same tap tree.

This encourages legal routing and edge sharing inside each clock tree.

### 5. Improvement rounds
After the initial solution is built, the solver performs a few improvement rounds before timeout:
- rip up selected weaker trees,
- increase history cost on problematic congested edges,
- reroute those trees using the updated penalties,
- keep the best solution found.

## Usage

```bash
python3 cts.py --input test.in --output out.txt
```

## Output format

For each tap, the solver outputs:
- assigned pins,
- routed edges for that tap tree.

Example:
```text
TAP 0
PINS 3
PIN 1
PIN 4
PIN 7
ROUTING 5
EDGE 1 1 1 2
EDGE 1 2 2 2
...
```

## Design goals

This implementation prioritizes:
1. legal connectivity,
2. avoiding capacity violations,
3. respecting tap load as much as possible,
4. reducing total routed wirelength,
5. improving score within limited runtime.

## Notes

- This is a heuristic solver, not an exact optimizer.
- Score improvement depends on testcase structure.
- The algorithm is tuned for better practical evaluator behavior compared with a simple nearest-assignment + single-pass routing baseline.

## Files

- `cts.py` — main solver
- `README.md` — project documentation
- `test.in` / `test0.in` — sample input
- `eval(1).py` — evaluator helper from project work

## Author

Law Sau Ho
