#!/usr/bin/env python3
"""
CENG4120 Clock Tree Synthesis (CTS) Project
Author: Law Sau Ho
Description: Assigns pins to taps and routes them with blockage avoidance.
"""
import argparse
import sys
from collections import defaultdict
from heapq import heappush, heappop
import time

NL = chr(10)

class CTSSolver:
    """Clock Tree Synthesis solver."""
    def __init__(self, max_runtime, max_load, grid_size, capacity):
        self.max_runtime = max_runtime if max_runtime is not None else 300
        self.max_load = max_load
        self.grid_size = grid_size
        self.capacity = capacity
        self.pins = []
        self.taps = []
        self.blockages = []
        self.tap_assignments = defaultdict(list)
        self.routing_edges = defaultdict(set)
        self.global_edge_usage = defaultdict(int)
        # Track which edges are already used by each tap (to avoid double-counting)
        self.tap_edge_sets = defaultdict(set)

    def add_pin(self, pin_id, x, y):
        self.pins.append({'id': pin_id, 'x': x, 'y': y})

    def add_tap(self, tap_id, x, y):
        self.taps.append({'id': tap_id, 'x': x, 'y': y})

    def add_blockage(self, blk_id, x1, y1, x2, y2):
        self.blockages.append({
            'id': blk_id,
            'x1': min(x1, x2), 'y1': min(y1, y2),
            'x2': max(x1, x2), 'y2': max(y1, y2),
        })

    def _edge_blocked(self, x1, y1, x2, y2):
        """True if edge passes THROUGH (not on boundary of) a blockage."""
        for b in self.blockages:
            if x1 == x2:
                # Vertical edge at x=x1, going from min(y1,y2) to max(y1,y2)
                if b['x1'] < x1 < b['x2']:
                    if min(y1, y2) < b['y2'] and max(y1, y2) > b['y1']:
                        return True
            else:
                # Horizontal edge at y=y1, going from min(x1,x2) to max(x1,x2)
                if b['y1'] < y1 < b['y2']:
                    if min(x1, x2) < b['x2'] and max(x1, x2) > b['x1']:
                        return True
        return False

    @staticmethod
    def _ekey(x1, y1, x2, y2):
        if (x1, y1) <= (x2, y2):
            return (x1, y1, x2, y2)
        return (x2, y2, x1, y1)

    def _edge_ok_for_tap(self, x1, y1, x2, y2, tap_idx):
        """Check if edge can be used (blockage + global capacity).
        Edges already used by this tap don't consume additional capacity.
        """
        if self._edge_blocked(x1, y1, x2, y2):
            return False
        k = self._ekey(x1, y1, x2, y2)
        # If this tap already uses this edge, it won't add to global usage
        if k in self.tap_edge_sets[tap_idx]:
            return True
        return self.global_edge_usage[k] < self.capacity

    def _find_path(self, sx, sy, ex, ey, tap_idx):
        """A* path avoiding blockages and respecting global capacity."""
        if sx == ex and sy == ey:
            return [(sx, sy)]
        gs = self.grid_size
        heap = []
        heappush(heap, (abs(ex - sx) + abs(ey - sy), 0, sx, sy))
        dist = {(sx, sy): 0}
        prev = {(sx, sy): None}
        while heap:
            f, g, cx, cy = heappop(heap)
            if cx == ex and cy == ey:
                path = []
                node = (cx, cy)
                while node is not None:
                    path.append(node)
                    node = prev[node]
                path.reverse()
                return path
            # Skip if we already found a better path to this node
            if g > dist.get((cx, cy), float('inf')):
                continue
            ng = g + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                # Bug fix: grid nodes go from 0 to gs inclusive (gs*gs grid has (gs+1)*(gs+1) nodes)
                if not (0 <= nx <= gs and 0 <= ny <= gs):
                    continue
                if not self._edge_ok_for_tap(cx, cy, nx, ny, tap_idx):
                    continue
                if ng < dist.get((nx, ny), float('inf')):
                    dist[(nx, ny)] = ng
                    prev[(nx, ny)] = (cx, cy)
                    h = abs(ex - nx) + abs(ey - ny)
                    heappush(heap, (ng + h, ng, nx, ny))
        return None

    def _assign_pins(self):
        """Greedy nearest-tap assignment."""
        if not self.taps:
            return
        tap_loads = [0] * len(self.taps)

        def nearest_dist(pi):
            px, py = self.pins[pi]['x'], self.pins[pi]['y']
            return min(abs(px - t['x']) + abs(py - t['y']) for t in self.taps)

        for pi in sorted(range(len(self.pins)), key=nearest_dist):
            px, py = self.pins[pi]['x'], self.pins[pi]['y']
            best_ti, best_d = None, float('inf')
            for ti, t in enumerate(self.taps):
                if tap_loads[ti] >= self.max_load:
                    continue
                d = abs(px - t['x']) + abs(py - t['y'])
                if d < best_d:
                    best_d, best_ti = d, ti
            if best_ti is not None:
                self.tap_assignments[best_ti].append(pi)
                tap_loads[best_ti] += 1
            else:
                # All taps are full: assign to least-loaded tap to avoid open pins
                min_load = min(tap_loads)
                for ti in range(len(self.taps)):
                    if tap_loads[ti] == min_load:
                        self.tap_assignments[ti].append(pi)
                        tap_loads[ti] += 1
                        break

    def _route_tap(self, tap_idx, deadline):
        """Build Steiner tree for one tap using Prim's algorithm."""
        pis = self.tap_assignments.get(tap_idx, [])
        if not pis:
            return
        t = self.taps[tap_idx]
        tx, ty = t['x'], t['y']
        connected = {(tx, ty)}
        remaining = list(range(len(pis)))
        while remaining:
            if time.time() > deadline:
                break
            best_path, best_idx, best_len = None, None, float('inf')
            for ri, pi_idx in enumerate(remaining):
                pin = self.pins[pis[pi_idx]]
                dx, dy = pin['x'], pin['y']
                cands = sorted(connected, key=lambda p: abs(p[0] - dx) + abs(p[1] - dy))
                for src in cands[:min(10, len(cands))]:
                    p = self._find_path(src[0], src[1], dx, dy, tap_idx)
                    if p is not None and len(p) < best_len:
                        best_len = len(p)
                        best_idx = ri
                        best_path = p
            if best_path is None:
                # Bug fix: no path found for any remaining pin.
                # Skip each unreachable pin one by one instead of aborting all.
                # Mark the first remaining pin as unroutable and remove it.
                remaining.pop(0)
                continue
            for i in range(len(best_path) - 1):
                ax, ay = best_path[i]
                bx, by = best_path[i + 1]
                k = self._ekey(ax, ay, bx, by)
                self.routing_edges[tap_idx].add(k)
                # Only increment global usage once per edge per tap
                if k not in self.tap_edge_sets[tap_idx]:
                    self.tap_edge_sets[tap_idx].add(k)
                    self.global_edge_usage[k] += 1
            # Add all path nodes to connected so they can be reused
            for node in best_path:
                connected.add(node)
            remaining.pop(best_idx)

    def solve(self):
        t0 = time.time()
        deadline = t0 + self.max_runtime - 0.5
        self._assign_pins()
        for ti in range(len(self.taps)):
            if time.time() > deadline:
                break
            self._route_tap(ti, deadline)

    def write_output(self, output_file):
        lines = []
        for ti, tap in enumerate(self.taps):
            tap_id = tap['id']
            pin_ids = [self.pins[pi]['id'] for pi in self.tap_assignments.get(ti, [])]
            edges = self.routing_edges.get(ti, set())
            lines.append('TAP ' + str(tap_id))
            lines.append('PINS ' + str(len(pin_ids)))
            for pid in pin_ids:
                lines.append('PIN ' + str(pid))
            lines.append('ROUTING ' + str(len(edges)))
            for (x1, y1, x2, y2) in edges:
                lines.append('EDGE ' + str(x1) + ' ' + str(y1) + ' ' + str(x2) + ' ' + str(y2))
        with open(output_file, 'w') as f:
            f.write(NL.join(lines) + NL)


def parse_input(input_file):
    with open(input_file, 'r') as f:
        data = f.read().split()
    it = iter(data)
    def nxt():
        return next(it)
    rt = ml = gs = cp = None
    solver = None
    try:
        while True:
            tok = nxt()
            if tok in ('MAXRUNTIME',):
                rt = int(nxt())
            elif tok in ('MAXLOAD', 'MAX_LOAD'):
                ml = int(nxt())
            elif tok in ('GRIDSIZE', 'GRID_SIZE'):
                gs = int(nxt())
            elif tok == 'CAPACITY':
                cp = int(nxt())
            elif tok == 'PINS':
                if ml is None or gs is None or cp is None:
                    print('WARNING: PINS encountered before required parameters.', file=sys.stderr)
                    continue
                solver = CTSSolver(rt, ml, gs, cp)
                n = int(nxt())
                for _ in range(n):
                    nxt()  # skip 'PIN' keyword
                    pid, px, py = int(nxt()), int(nxt()), int(nxt())
                    solver.add_pin(pid, px, py)
            elif tok == 'TAPS':
                if solver is None:
                    print('WARNING: TAPS encountered before PINS.', file=sys.stderr)
                    continue
                n = int(nxt())
                for _ in range(n):
                    nxt()  # skip 'TAP' keyword
                    tid, tx, ty = int(nxt()), int(nxt()), int(nxt())
                    solver.add_tap(tid, tx, ty)
            elif tok == 'BLKS':
                if solver is None:
                    print('WARNING: BLKS encountered before PINS.', file=sys.stderr)
                    continue
                n = int(nxt())
                for _ in range(n):
                    nxt()  # skip 'BLK' keyword
                    bid = int(nxt())
                    x1, y1 = int(nxt()), int(nxt())
                    x2, y2 = int(nxt()), int(nxt())
                    solver.add_blockage(bid, x1, y1, x2, y2)
    except StopIteration:
        pass
    except (ValueError, AttributeError) as e:
        print(f'WARNING: Parse error: {e}', file=sys.stderr)
    if solver is not None:
        return solver
    if ml is not None and gs is not None and cp is not None:
        return CTSSolver(rt, ml, gs, cp)
    return None


def main():
    parser = argparse.ArgumentParser(description='CENG4120 CTS Solver')
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    solver = parse_input(args.input)
    if solver is None:
        print('ERROR: Failed to parse input.', file=sys.stderr)
        sys.exit(1)
    solver.solve()
    solver.write_output(args.output)


if __name__ == '__main__':
    main()
