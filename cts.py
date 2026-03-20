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


class CTSSolver:
    """Clock Tree Synthesis solver"""

    def __init__(self, max_runtime, max_load, grid_size, capacity):
        self.max_runtime = max_runtime
        self.max_load = max_load
        self.grid_size = grid_size
        self.capacity = capacity
        self.pins = []          # list of {'id': int, 'x': int, 'y': int}
        self.taps = []          # list of {'id': int, 'x': int, 'y': int}
        self.blockages = []     # list of {'id', 'x1','y1','x2','y2'}
        self.tap_assignments = defaultdict(list)   # tap_idx -> [pin_idx, ...]
        # tap_idx -> set of canonical edge tuples (x1,y1,x2,y2) unit segments
        self.routing_edges = defaultdict(set)
        self.edge_usage = defaultdict(int)  # edge_key -> usage count

    # ------------------------------------------------------------------
    # Input helpers
    # ------------------------------------------------------------------
    def add_pin(self, pin_id, x, y):
        self.pins.append({'id': pin_id, 'x': x, 'y': y})

    def add_tap(self, tap_id, x, y):
        self.taps.append({'id': tap_id, 'x': x, 'y': y})

    def add_blockage(self, blk_id, x1, y1, x2, y2):
        self.blockages.append({
            'id': blk_id,
            'x1': min(x1, x2), 'y1': min(y1, y2),
            'x2': max(x1, x2), 'y2': max(y1, y2)
        })

    # ------------------------------------------------------------------
    # Blockage / capacity helpers
    # ------------------------------------------------------------------
    def _edge_blocked(self, x1, y1, x2, y2):
        """Return True if the unit edge from (x1,y1) to (x2,y2) is blocked."""
        for b in self.blockages:
            if x1 == x2:  # vertical edge
                ey_lo, ey_hi = min(y1, y2), max(y1, y2)
                if b['x1'] <= x1 <= b['x2'] and b['y1'] < ey_hi and b['y2'] > ey_lo:
                    return True
            else:          # horizontal edge
                ex_lo, ex_hi = min(x1, x2), max(x1, x2)
                if b['y1'] <= y1 <= b['y2'] and b['x1'] < ex_hi and b['x2'] > ex_lo:
                    return True
        return False

    @staticmethod
    def _ekey(x1, y1, x2, y2):
        """Canonical edge key (smaller coord first)."""
        if (x1, y1) <= (x2, y2):
            return (x1, y1, x2, y2)
        return (x2, y2, x1, y1)

    def _edge_available(self, x1, y1, x2, y2):
        """True if the edge can be used (not blocked, not over-capacity)."""
        if self._edge_blocked(x1, y1, x2, y2):
            return False
        k = self._ekey(x1, y1, x2, y2)
        if self.edge_usage[k] >= self.capacity:
            return False
        return True

    # ------------------------------------------------------------------
    # Pathfinding  (BFS for exact shortest path, A* heuristic order)
    # ------------------------------------------------------------------
    def _find_path(self, sx, sy, ex, ey):
        """
        A* shortest Manhattan path from (sx,sy) to (ex,ey).
        Returns list of (x, y) tuples including start and end,
        or None if no path exists.
        """
        if sx == ex and sy == ey:
            return [(sx, sy)]

        gs = self.grid_size
        open_heap = []   # (f, g, x, y)
        heappush(open_heap, (abs(ex - sx) + abs(ey - sy), 0, sx, sy))
        visited = {}     # (x,y) -> (parent_x, parent_y) or None for start
        visited[(sx, sy)] = None

        while open_heap:
            f, g, cx, cy = heappop(open_heap)
            if cx == ex and cy == ey:
                # Reconstruct path
                path = []
                node = (cx, cy)
                while node is not None:
                    path.append(node)
                    node = visited[node]
                path.reverse()
                return path
            ng = g + 1
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx <= gs and 0 <= ny <= gs):
                    continue
                if (nx, ny) in visited:
                    continue
                if not self._edge_available(cx, cy, nx, ny):
                    continue
                h = abs(ex - nx) + abs(ey - ny)
                visited[(nx, ny)] = (cx, cy)
                heappush(open_heap, (ng + h, ng, nx, ny))
        return None

    def _manhattan_path(self, sx, sy, ex, ey):
        """Simple L-shaped path ignoring capacity/blockages (fallback)."""
        path = [(sx, sy)]
        cx, cy = sx, sy
        while cx != ex:
            cx += 1 if ex > cx else -1
            path.append((cx, cy))
        while cy != ey:
            cy += 1 if ey > cy else -1
            path.append((cx, cy))
        return path

    # ------------------------------------------------------------------
    # Pin assignment
    # ------------------------------------------------------------------
    def _assign_pins(self):
        """
        Greedy assignment: sort pins by distance to nearest tap,
        assign each to the closest tap that still has capacity.
        """
        tap_loads = [0] * len(self.taps)

        def nearest_tap_dist(pi):
            px, py = self.pins[pi]['x'], self.pins[pi]['y']
            return min(abs(px - t['x']) + abs(py - t['y']) for t in self.taps)

        pin_order = sorted(range(len(self.pins)), key=nearest_tap_dist)

        for pi in pin_order:
            px, py = self.pins[pi]['x'], self.pins[pi]['y']
            best_ti, best_d = None, float('inf')
            for ti, tap in enumerate(self.taps):
                if tap_loads[ti] >= self.max_load:
                    continue
                d = abs(px - tap['x']) + abs(py - tap['y'])
                if d < best_d:
                    best_d = d
                    best_ti = ti
            if best_ti is not None:
                self.tap_assignments[best_ti].append(pi)
                tap_loads[best_ti] += 1

    # ------------------------------------------------------------------
    # Routing per tap
    # ------------------------------------------------------------------
    def _route_tap(self, tap_idx, deadline):
        """
        Build a Steiner-like routing tree connecting the tap to all its pins.
        Uses Prim's: grow a connected set, connect the closest remaining pin.
        """
        pin_indices = self.tap_assignments.get(tap_idx, [])
        if not pin_indices:
            return

        tap = self.taps[tap_idx]
        tx, ty = tap['x'], tap['y']

        # connected set: set of (x,y) already in tree
        connected = {(tx, ty)}
        remaining = [(self.pins[pi]['x'], self.pins[pi]['y']) for pi in pin_indices]

        while remaining:
            if time.time() > deadline:
                break

            best_src = None
            best_dst = None
            best_path = None
            best_len = float('inf')

            for dst in remaining:
                dx, dy = dst
                # try to connect from nearest already-connected point
                candidates = sorted(connected,
                                    key=lambda p: abs(p[0]-dx)+abs(p[1]-dy))
                # only try a few nearest candidates for speed
                for src in candidates[:5]:
                    path = self._find_path(src[0], src[1], dx, dy)
                    if path is not None and len(path) < best_len:
                        best_len = len(path)
                        best_src = src
                        best_dst = dst
                        best_path = path
                        if best_len == 1:
                            break

            if best_path is None:
                # fallback: Manhattan path (may violate capacity)
                dst = remaining[0]
                best_path = self._manhattan_path(tx, ty, dst[0], dst[1])
                best_dst = dst

            # Record edges
            for i in range(len(best_path) - 1):
                ax, ay = best_path[i]
                bx, by = best_path[i + 1]
                k = self._ekey(ax, ay, bx, by)
                self.routing_edges[tap_idx].add(k)
                self.edge_usage[k] += 1
                connected.add((bx, by))

            remaining.remove(best_dst)

    # ------------------------------------------------------------------
    # Main solve
    # ------------------------------------------------------------------
    def solve(self):
        start = time.time()
        deadline = start + self.max_runtime - 0.5

        self._assign_pins()

        for ti in range(len(self.taps)):
            if time.time() > deadline:
                break
            self._route_tap(ti, deadline)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def write_output(self, output_file):
        with open(output_file, 'w') as f:
            for ti, tap in enumerate(self.taps):
                tap_id = tap['id']
                pin_ids = [self.pins[pi]['id'] for pi in self.tap_assignments.get(ti, [])]
                edges = self.routing_edges.get(ti, set())

                f.write(f"TAP {tap_id}\n")
                f.write(f"PINS {len(pin_ids)}\n")
                for pid in pin_ids:
                    f.write(f"PIN {pid}\n")
                f.write(f"ROUTING {len(edges)}\n")
                for (x1, y1, x2, y2) in edges:
                    f.write(f"EDGE {x1} {y1} {x2} {y2}\n")


# ---------------------------------------------------------------------------
# Input parser
# ---------------------------------------------------------------------------
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
            if tok == 'MAXRUNTIME':
                rt = int(nxt())
            elif tok == 'MAXLOAD':
                ml = int(nxt())
            elif tok == 'GRIDSIZE':
                gs = int(nxt())
            elif tok == 'CAPACITY':
                cp = int(nxt())
                solver = CTSSolver(rt, ml, gs, cp)
            elif tok == 'PINS':
                n = int(nxt())
                for _ in range(n):
                    _kw = nxt()  # 'PIN'
                    pid, px, py = int(nxt()), int(nxt()), int(nxt())
                    solver.add_pin(pid, px, py)
            elif tok == 'TAPS':
                n = int(nxt())
                for _ in range(n):
                    _kw = nxt()  # 'TAP'
                    tid, tx, ty = int(nxt()), int(nxt()), int(nxt())
                    solver.add_tap(tid, tx, ty)
            elif tok == 'BLKS':
                n = int(nxt())
                for _ in range(n):
                    _kw = nxt()  # 'BLK'
                    bid = int(nxt())
                    x1, y1, x2, y2 = int(nxt()), int(nxt()), int(nxt()), int(nxt())
                    solver.add_blockage(bid, x1, y1, x2, y2)
    except StopIteration:
        pass

    return solver


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='CENG4120 CTS Solver')
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    solver = parse_input(args.input)
    if solver is None:
        print("ERROR: Failed to parse input.", file=sys.stderr)
        return
    solver.solve()
    solver.write_output(args.output)


if __name__ == '__main__':
    main()
