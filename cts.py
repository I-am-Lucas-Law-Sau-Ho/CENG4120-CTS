#!/usr/bin/env python3
"""
CENG4120 Clock Tree Synthesis (CTS) Project
Author: Law Sau Ho
Description: Clock tree synthesis implementation with optimized routing
"""

import argparse
import sys
from collections import defaultdict
from heapq import heappush, heappop
import time

class Point:
    """Represents a 2D coordinate"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def manhattan_distance(self, other):
        return abs(self.x - other.x) + abs(self.y - other.y)
    
    def __repr__(self):
        return f"({self.x},{self.y})"

class Edge:
    """Represents a routing edge"""
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
    
    def get_horizontal_edges(self):
        """Returns all horizontal unit edges"""
        edges = []
        if self.p1.y == self.p2.y:
            x1, x2 = min(self.p1.x, self.p2.x), max(self.p1.x, self.p2.x)
            for x in range(x1, x2):
                edges.append((Point(x, self.p1.y), Point(x+1, self.p1.y)))
        return edges
    
    def get_vertical_edges(self):
        """Returns all vertical unit edges"""
        edges = []
        if self.p1.x == self.p2.x:
            y1, y2 = min(self.p1.y, self.p2.y), max(self.p1.y, self.p2.y)
            for y in range(y1, y2):
                edges.append((Point(self.p1.x, y), Point(self.p1.x, y+1)))
        return edges
    
    def get_all_unit_edges(self):
        """Returns all unit edges"""
        return self.get_horizontal_edges() + self.get_vertical_edges()

class CTSSolver:
    """Clock Tree Synthesis solver"""
    
    def __init__(self, max_runtime, max_load, grid_size, capacity):
        self.max_runtime = max_runtime
        self.max_load = max_load
        self.grid_size = grid_size
        self.capacity = capacity
        self.pins = []
        self.taps = []
        self.blockages = []
        self.tap_assignments = defaultdict(list)  # tap_id -> list of pin_ids
        self.routing_edges = defaultdict(set)     # tap_id -> set of edges
        self.edge_usage = defaultdict(int)        # edge -> count
        
    def add_pin(self, pin_id, x, y):
        self.pins.append({'id': pin_id, 'pos': Point(x, y)})
    
    def add_tap(self, tap_id, x, y):
        self.taps.append({'id': tap_id, 'pos': Point(x, y)})
    
    def add_blockage(self, blk_id, x1, y1, x2, y2):
        self.blockages.append({
            'id': blk_id,
            'x1': min(x1, x2),
            'y1': min(y1, y2),
            'x2': max(x1, x2),
            'y2': max(y1, y2)
        })
    
    def is_blocked(self, p1, p2):
        """Check if edge intersects with any blockage"""
        for blk in self.blockages:
            if p1.x == p2.x:  # Vertical edge
                x = p1.x
                y1, y2 = min(p1.y, p2.y), max(p1.y, p2.y)
                if blk['x1'] <= x <= blk['x2']:
                    if not (y2 <= blk['y1'] or y1 >= blk['y2']):
                        return True
            else:  # Horizontal edge
                y = p1.y
                x1, x2 = min(p1.x, p2.x), max(p1.x, p2.x)
                if blk['y1'] <= y <= blk['y2']:
                    if not (x2 <= blk['x1'] or x1 >= blk['x2']):
                        return True
        return False
    
    def find_path(self, start, end, used_edges):
        """A* pathfinding avoiding blockages and respecting capacity"""
        open_set = []
        heappush(open_set, (0, start, []))
        visited = set()
        
        while open_set:
            f_score, current, path = heappop(open_set)
            
            if current == end:
                return path + [current]
            
            if current in visited:
                continue
            visited.add(current)
            
            # Try all 4 directions
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = current.x + dx, current.y + dy
                if 0 <= nx <= self.grid_size and 0 <= ny <= self.grid_size:
                    neighbor = Point(nx, ny)
                    
                    # Check blockage
                    if self.is_blocked(current, neighbor):
                        continue
                    
                    # Check capacity
                    edge_key = tuple(sorted([(current.x, current.y), (neighbor.x, neighbor.y)]))
                    if edge_key in used_edges and used_edges[edge_key] >= self.capacity:
                        continue
                    
                    g_score = len(path) + 1
                    h_score = neighbor.manhattan_distance(end)
                    f = g_score + h_score
                    
                    heappush(open_set, (f, neighbor, path + [current]))
        
        return None  # No path found
    
    def assign_pins_to_taps(self):
        """Assign each pin to the nearest available tap"""
        unassigned_pins = list(range(len(self.pins)))
        tap_loads = [0] * len(self.taps)
        
        # Sort pins by distance to nearest tap
        def get_min_tap_distance(pin_idx):
            pin_pos = self.pins[pin_idx]['pos']
            return min(pin_pos.manhattan_distance(tap['pos']) for tap in self.taps)
        
        unassigned_pins.sort(key=get_min_tap_distance)
        
        for pin_idx in unassigned_pins:
            pin = self.pins[pin_idx]
            pin_pos = pin['pos']
            
            # Find best available tap
            best_tap = None
            best_distance = float('inf')
            
            for tap_idx, tap in enumerate(self.taps):
                if tap_loads[tap_idx] < self.max_load:
                    dist = pin_pos.manhattan_distance(tap['pos'])
                    if dist < best_distance:
                        best_distance = dist
                        best_tap = tap_idx
            
            if best_tap is not None:
                self.tap_assignments[best_tap].append(pin_idx)
                tap_loads[best_tap] += 1
    
    def route_tap_tree(self, tap_idx):
        """Route all pins assigned to a tap using Steiner tree approximation"""
        if tap_idx not in self.tap_assignments or len(self.tap_assignments[tap_idx]) == 0:
            return
        
        tap_pos = self.taps[tap_idx]['pos']
        pin_indices = self.tap_assignments[tap_idx]
        
        # Start from tap position
        connected = {tap_pos}
        edges = set()
        local_edge_usage = defaultdict(int)
        
        # Connect each pin to the nearest connected point (Prim's algorithm variant)
        remaining_pins = [self.pins[idx]['pos'] for idx in pin_indices]
        
        while remaining_pins:
            best_pin = None
            best_path = None
            best_distance = float('inf')
            
            for pin_pos in remaining_pins:
                for connected_pos in connected:
                    path = self.find_path(connected_pos, pin_pos, local_edge_usage)
                    if path and len(path) < best_distance:
                        best_distance = len(path)
                        best_pin = pin_pos
                        best_path = path
            
            if best_path is None:
                # Fallback: direct Manhattan path
                pin_pos = remaining_pins[0]
                best_path = self.manhattan_path(tap_pos, pin_pos)
                best_pin = pin_pos
            
            # Add edges from path
            for i in range(len(best_path) - 1):
                p1, p2 = best_path[i], best_path[i+1]
                edge_key = tuple(sorted([(p1.x, p1.y), (p2.x, p2.y)]))
                edges.add((p1, p2))
                local_edge_usage[edge_key] += 1
                self.edge_usage[edge_key] += 1
                connected.add(p2)
            
            remaining_pins.remove(best_pin)
        
        self.routing_edges[tap_idx] = edges
    
    def manhattan_path(self, start, end):
        """Generate simple Manhattan routing path"""
        path = [start]
        current = Point(start.x, start.y)
        
        # Go horizontal first
        while current.x != end.x:
            current = Point(current.x + (1 if end.x > current.x else -1), current.y)
            path.append(current)
        
        # Then vertical
        while current.y != end.y:
            current = Point(current.x, current.y + (1 if end.y > current.y else -1))
            path.append(current)
        
        return path
    
    def solve(self):
        """Main solving function"""
        start_time = time.time()
        
        # Step 1: Assign pins to taps
        self.assign_pins_to_taps()
        
        # Step 2: Route each tap's tree
        for tap_idx in range(len(self.taps)):
            if time.time() - start_time > self.max_runtime - 0.5:
                break
            self.route_tap_tree(tap_idx)
        
        return True
    
    def write_output(self, output_file):
        """Write solution to output file"""
        with open(output_file, 'w') as f:
            for tap_idx in range(len(self.taps)):
                tap_id = self.taps[tap_idx]['id']
                f.write(f"TAP {tap_id}\\n")
                
                # Write assigned pins
                pin_ids = [self.pins[idx]['id'] for idx in self.tap_assignments.get(tap_idx, [])]
                f.write(f"PINS {len(pin_ids)}\\n")
                for pin_id in pin_ids:
                    f.write(f"PIN {pin_id}\\n")
                
                # Write routing edges
                edges = self.routing_edges.get(tap_idx, set())
                f.write(f"ROUTING {len(edges)}\\n")
                for p1, p2 in edges:
                    f.write(f"EDGE {p1.x} {p1.y} {p2.x} {p2.y}\\n")

def parse_input(input_file):
    """Parse input file and return solver instance"""
    with open(input_file, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    
    solver = None
    i = 0
    
    while i < len(lines):
        parts = lines[i].split()
        
        if parts[0] == 'MAXRUNTIME':
            max_runtime = int(parts[1])
            i += 1
        elif parts[0] == 'MAXLOAD':
            max_load = int(parts[1])
            i += 1
        elif parts[0] == 'GRIDSIZE':
            grid_size = int(parts[1])
            i += 1
        elif parts[0] == 'CAPACITY':
            capacity = int(parts[1])
            solver = CTSSolver(max_runtime, max_load, grid_size, capacity)
            i += 1
        elif parts[0] == 'PINS':
            num_pins = int(parts[1])
            i += 1
            for _ in range(num_pins):
                pin_parts = lines[i].split()
                pin_id = int(pin_parts[1])
                x, y = int(pin_parts[2]), int(pin_parts[3])
                solver.add_pin(pin_id, x, y)
                i += 1
        elif parts[0] == 'TAPS':
            num_taps = int(parts[1])
            i += 1
            for _ in range(num_taps):
                tap_parts = lines[i].split()
                tap_id = int(tap_parts[1])
                x, y = int(tap_parts[2]), int(tap_parts[3])
                solver.add_tap(tap_id, x, y)
                i += 1
        elif parts[0] == 'BLKS':
            num_blks = int(parts[1])
            i += 1
            for _ in range(num_blks):
                blk_parts = lines[i].split()
                blk_id = int(blk_parts[1])
                x1, y1 = int(blk_parts[2]), int(blk_parts[3])
                x2, y2 = int(blk_parts[4]), int(blk_parts[5])
                solver.add_blockage(blk_id, x1, y1, x2, y2)
                i += 1
        else:
            i += 1
    
    return solver

def main():
    parser = argparse.ArgumentParser(description='CENG4120 Clock Tree Synthesis')
    parser.add_argument('--input', required=True, help='Input file path')
    parser.add_argument('--output', required=True, help='Output file path')
    args = parser.parse_args()
    
    # Parse input
    solver = parse_input(args.input)
    
    # Solve
    solver.solve()
    
    # Write output
    solver.write_output(args.output)
    
    print(f"CTS completed. Output written to {args.output}")

if __name__ == "__main__":
    main()
