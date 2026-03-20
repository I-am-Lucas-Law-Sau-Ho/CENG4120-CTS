#!/usr/bin/env python3
"""
CENG4120 Clock Tree Synthesis (CTS) Project
Author: Law Sau Ho
Description: Clock tree synthesis - assigns pins to taps and routes them.
"""

import argparse
import sys
from collections import defaultdict
from heapq import heappush, heappop
import time

NL = chr(10)  # newline character


class CTSSolver:
    """Clock Tree Synthesis solver."""

    def __init__(self, max_runtime, max_load, grid_size, capacity):
        self.max_runtime = max_runtime
        self.max_load = max_load
        self.grid_size = grid_size
        self.capacity = capacity
        self.pins = []       # [{'id', 'x', 'y'}, ...]
        self.taps = []       # [{'id', 'x', 'y'}, ...]
        self.blockages = []  # [{'id','x1','y1','x2','y2'}, ...]
        self.tap_assignments = defaultdict(list)  # tap_idx -> [pin_idx,...]
        self.routing_edges = defaultdict(set)     # tap_idx -> {(x1,y1,x2,y2)}
        self.edge_usage = defaultdict(int)        # ekey -> count

    # --- input -------------------------------------------------------
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

    # --- blockage / capacity -----------------------------------------
    def _edge_blocked(self, x1, y1, x2, y2):
        """True if the unit edge (x1,y1)-(x2,y2) passes THROUGH a blockage."""
        for b in self.blockages:
            if x1 == x2:  # vertical edge
                ey_lo = min(y1, y2)
                ey_hi = max(y1, y2)
                # edge is strictly inside blockage in x?
                if b['x1'] < x1 < b['x2']:
                    # overlaps in y?
                    if ey_lo < b['y2'] and ey_hi > b['y1']:
                        return True
            else:         # horizontal edge
                ex_lo = min(x1, x2)
                ex_hi = max(x1, x2)
                if b['y1'] < y1 < b['y2']:
                    if ex_lo < b['x2'] and ex_hi > b['x1']:
                        return True
        return False

    @staticmethod
    def _ekey(x1, y1, x2, y2):
        """Canonical edge key."""
        if (x1, y1) <= (x2, y2):
            return (x1, y1, x2, y2)
        return (x2, y2, x1, y1)

    def _edge_ok(self, x1, y1, x2, y2):
        """True if edge can be used."""
        if self._edge_blocked(x1, y1, x2, y2):
            return False
        return self.edge_usage[self._ekey(x1, y1, x2, y2)] < self.capacity

    # --- A* pathfinding ----------------------------------------------
    def _find_path(self, sx, sy, ex, ey):
        """A* path. Returns [(x,y),...] or None."""
        if sx == ex and sy == ey:
            return [(sx, sy)]
        gs = self.grid_size
        heap = []
        heappush(heap, (abs(ex-sx)+abs(ey-sy), 0, sx, sy))
        prev = {(sx, sy): None}
        while heap:
            _, g, cx, cy = heappop(heap)
            if cx == ex and cy == ey:
                path = []
                node = (cx, cy)
                while node is not None:
                    path.append(node)
                    node = prev[node]
                path.reverse()
                return path
            ng = g + 1
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = cx+dx, cy+dy
                if not (0 <= nx <= gs and 0 <= ny <= gs):
                    continue
                if (nx, ny) in prev:
                    continue
                if not self._edge_ok(cx, cy, nx, ny):
                    continue
                prev[(nx, ny)] = (cx, cy)
                h = abs(ex-nx)+abs(ey-ny)
                heappush(heap, (ng+h, ng, nx, ny))
        return None

    def _l_path(self, sx, sy, ex, ey):
        """Simple L-shaped fallback path."""
        path = [(sx, sy)]
        cx, cy = sx, sy
        while cx != ex:
            cx += 1 if ex > cx else -1
            path.append((cx, cy))
        while cy != ey:
            cy += 1 if ey > cy else -1
            path.append((cx, cy))
        return path

    # --- pin assignment ----------------------------------------------
    def _assign_pins(self):
        """Greedy nearest-tap assignment respecting max_load."""
        tap_loads = [0] * len(self.taps)

        def nearest_dist(pi):
            px, py = self.pins[pi]['x'], self.pins[pi]['y']
            return min(abs(px-t['x'])+abs(py-t['y']) for t in self.taps)

        for pi in sorted(range(len(self.pins)), key=nearest_dist):
            px, py = self.pins[pi]['x'], self.pins[pi]['y']
            best_ti, best_d = None, float('inf')
            for ti, t in enumerate(self.taps):
                if tap_loads[ti] >= self.max_load:
                    continue
                d = abs(px-t['x'])+abs(py-t['y'])
                if d < best_d:
                    best_d, best_ti = d, ti
            if best_ti is not None:
                self.tap_assignments[best_ti].append(pi)
                tap_loads[best_ti] += 1

    # --- routing per tap ---------------------------------------------
    def _route_tap(self, tap_idx, deadline):
        """Prim-style Steiner tree for one tap."""
        pis = self.tap_assignments.get(tap_idx, [])
        if not pis:
            return
        t = self.taps[tap_idx]
        tx, ty = t['x'], t['y']
        connected = {(tx, ty)}
        remaining = [(self.pins[pi]['x'], self.pins[pi]['y']) for pi in pis]

        while remaining:
            if time.time() > deadline:
                break
            best_path, best_dst, best_len = None, None, float('inf')
            for dst in remaining:
                dx, dy = dst
                cands = sorted(connected, key=lambda p: abs(p[0]-dx)+abs(p[1]-dy))
                for src in cands[:5]:
                    p = self._find_path(src[0], src[1], dx, dy)
                    if p is not None and len(p) < best_len:
                        best_len, best_dst, best_path = len(p), dst, p
            if best_path is None:
                best_dst = remaining[0]
                best_path = self._l_path(tx, ty, best_dst[0], best_dst[1])
            for i in range(len(best_path)-1):
                ax, ay = best_path[i]
                bx, by = best_path[i+1]
                k = self._ekey(ax, ay, bx, by)
                self.routing_edges[tap_idx].add(k)
                self.edge_usage[k] += 1
                connected.add((bx, by))
            remaining.remove(best_dst)

    # --- main solve --------------------------------------------------
    def solve(self):
        t0 = time.time()
        deadline = t0 + self.max_runtime - 0.5
        self._assign_pins()
        for ti in range(len(self.taps)):
            if time.time() > deadline:
                break
            self._route_tap(ti, deadline)

    # --- output -------------------------------------------------------
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
                    nxt()  # 'PIN'
                    pid, px, py = int(nxt()), int(nxt()), int(nxt())
                    solver.add_pin(pid, px, py)
            elif tok == 'TAPS':
                n = int(nxt())
                for _ in range(n):
                    nxt()  # 'TAP'
                    tid, tx, ty = int(nxt()), int(nxt()), int(nxt())
                    solver.add_tap(tid, tx, ty)
            elif tok == 'BLKS':
                n = int(nxt())
                for _ in range(n):
                    nxt()  # 'BLK'
                    bid = int(nxt())
                    x1, y1 = int(nxt()), int(nxt())
                    x2, y2 = int(nxt()), int(nxt())
                    solver.add_blockage(bid, x1, y1, x2, y2)
    except (StopIteration, ValueError, AttributeError):
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
        print('ERROR: Failed to parse input.', file=sys.stderr)
        sys.exit(1)
    solver.solve()
    solver.write_output(args.output)


if __name__ == '__main__':
    main()
