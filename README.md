# CENG4120 Clock Tree Synthesis (CTS)

## Project Overview
This project implements a Clock Tree Synthesis (CTS) solver for the CENG4120 course at CUHK. The program assigns clock pins to clock taps and routes them while minimizing delay skew and total wire length.

## Author
Law Sau Ho (Lucas)
CUHK Computer Engineering
March 2026

## Features
- **Pin Assignment**: Optimal assignment of pins to taps based on Manhattan distance
- **A* Pathfinding**: Intelligent routing that avoids blockages and respects capacity constraints
- **Steiner Tree Construction**: Efficient tree construction using Prim's algorithm variant
- **Constraint Satisfaction**: Respects MAX_LOAD and CAPACITY constraints
- **Blockage Avoidance**: Routes around rectangular obstacles
- **Cost Optimization**: Minimizes delay skew and total tree length

## Requirements
- Python 3.6 or higher
- No external dependencies (uses only standard library)

## Installation
```bash
git clone https://github.com/I-am-Lucas-Law-Sau-Ho/CENG4120-CTS.git
cd CENG4120-CTS
chmod +x cts.py
```

## Usage
```bash
python3 cts.py --input <input_file> --output <output_file>
```

Or make it executable:
```bash
chmod +x cts.py
./cts.py --input test.in --output test.out
```

## Input Format
```
MAXRUNTIME <time_in_seconds>
MAXLOAD <max_pins_per_tap>
GRIDSIZE <grid_dimension>
CAPACITY <max_wires_per_edge>
PINS <num_pins>
PIN <id> <x> <y>
...
TAPS <num_taps>
TAP <id> <x> <y>
...
BLKS <num_blockages>
BLK <id> <x1> <y1> <x2> <y2>
...
```

## Output Format
```
TAP <tap_id>
PINS <num_pins>
PIN <pin_id>
...
ROUTING <num_edges>
EDGE <x1> <y1> <x2> <y2>
...
```

## Algorithm Details

### 1. Pin Assignment
- Sorts pins by distance to nearest tap
- Greedily assigns each pin to the closest available tap
- Respects MAX_LOAD constraint

### 2. Routing
- Uses A* pathfinding for each connection
- Heuristic: Manhattan distance to target
- Avoids blockages and respects capacity constraints
- Builds Steiner tree using Prim's algorithm variant

### 3. Optimization
The cost function minimizes:
```
cost = (max_delay - min_delay) / num_taps + sum(tree_lengths) / num_taps
```

## Example
Sample input (`test.in`):
```
MAXRUNTIME 1
MAXLOAD 5
GRIDSIZE 10
CAPACITY 1
PINS 8
PIN 0 0 7
PIN 1 8 7
PIN 2 6 9
PIN 3 1 2
PIN 4 2 1
PIN 5 9 0
PIN 6 1 0
PIN 7 7 8
TAPS 2
TAP 0 4 5
TAP 1 5 2
```

Run:
```bash
./cts.py --input test.in --output test.out
```

## Project Structure
```
CENG4120-CTS/
├── cts.py          # Main solver implementation
└── README.md       # This file
```

## Implementation Notes
- **Point class**: Represents 2D coordinates with Manhattan distance calculation
- **Edge class**: Manages routing edges and unit edge decomposition
- **CTSSolver class**: Main solver with pin assignment and routing logic
- **Time Management**: Stops routing if approaching MAXRUNTIME limit
- **Edge Merging**: Overlapping edges are automatically merged in output

## Evaluation Criteria
According to project specification:
- **Connectivity Check** (20%): No open or short pins
- **Capacity Check** (20%): Respect edge capacity constraints
- **Load Check** (20%): Each tap drives ≤ MAX_LOAD pins
- **No Open/Short** (30%): All pins properly connected
- **Ranking Bonus** (10%): Based on cost function performance

## Testing
To verify your solution:
1. Create test input files following the format above
2. Run the solver
3. Check output for:
   - All pins assigned
   - No capacity violations
   - All pins connected to taps
   - Valid edge coordinates

## Troubleshooting

### Common Issues
1. **"No path found"**: Increase grid size or reduce blockages
2. **Capacity violations**: Increase CAPACITY or reduce pin density
3. **Load violations**: Add more taps or increase MAXLOAD

### Performance Tips
- Smaller grid sizes run faster
- Fewer blockages enable better routing
- More taps distribute load better

## Academic Integrity
This project was completed as part of CENG4120 course requirements. The implementation uses standard algorithms (A*, Prim's, greedy assignment) adapted for the CTS problem.

## License
This project is submitted as coursework for CENG4120 at CUHK.

## Contact
Law Sau Ho
Computer Engineering, CUHK
Email: lucaslawsauho@gmail.com

---
*Last Updated: March 20, 2026*
