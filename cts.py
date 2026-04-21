#!/usr/bin/env python3
import argparse
import time
from collections import defaultdict
from heapq import heappush, heappop

class CTSSolver:
    def __init__(self, max_runtime, max_load, grid_size, capacity):
        self.max_runtime = max_runtime if max_runtime is not None else 300
        self.max_load = max_load
        self.grid_size = grid_size
        self.capacity = capacity
        self.pins, self.taps, self.blockages = [], [], []
        self.tap_assignments = defaultdict(list)
        self.routing_edges = defaultdict(set)
        self.tap_edge_sets = defaultdict(set)
        self.global_edge_usage = defaultdict(int)
        self.blocked_edges = set()
        self.history_cost = defaultdict(float)
        self.best_snapshot = None

    def add_pin(self, pid, x, y): self.pins.append({'id': pid, 'x': x, 'y': y})
    def add_tap(self, tid, x, y): self.taps.append({'id': tid, 'x': x, 'y': y})
    def add_blockage(self, bid, x1, y1, x2, y2):
        self.blockages.append({'x1': min(x1, x2), 'y1': min(y1, y2), 'x2': max(x1, x2), 'y2': max(y1, y2)})

    @staticmethod
    def _ekey(x1, y1, x2, y2):
        return (x1, y1, x2, y2) if (x1, y1) <= (x2, y2) else (x2, y2, x1, y1)

    def _precompute_blocked_edges(self):
        for b in self.blockages:
            x1, y1, x2, y2 = b['x1'], b['y1'], b['x2'], b['y2']
            for x in range(x1, x2):
                for y in range(y1, y2 + 1): self.blocked_edges.add(self._ekey(x, y, x + 1, y))
            for x in range(x1, x2 + 1):
                for y in range(y1, y2): self.blocked_edges.add(self._ekey(x, y, x, y + 1))

    def _find_path(self, sx, sy, ex, ey, tap_idx):
        if sx == ex and sy == ey: return [(sx, sy)]
        gs = self.grid_size
        heap = [(abs(ex - sx) + abs(ey - sy), 0.0, sx, sy)]
        dist = defaultdict(lambda: float('inf'), {(sx, sy): 0.0})
        prev = {(sx, sy): None}
        while heap:
            f, g, cx, cy = heappop(heap)
            if (cx, cy) == (ex, ey):
                path = []
                while (cx, cy) in prev:
                    path.append((cx, cy))
                    res = prev[(cx, cy)]
                    if res is None: break
                    cx, cy = res
                return path[::-1]
            if g > dist[(cx, cy)]: continue
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < gs and 0 <= ny < gs:
                    k = self._ekey(cx, cy, nx, ny)
                    if k in self.blocked_edges: continue
                    if k not in self.tap_edge_sets[tap_idx] and self.global_edge_usage[k] >= self.capacity: continue
                    cost = 0.05 if k in self.tap_edge_sets[tap_idx] else (1.0 + 10.0 * (self.global_edge_usage[k]/self.capacity) + self.history_cost[k])
                    ng = g + cost
                    if ng < dist[(nx, ny)]:
                        dist[(nx, ny)] = ng
                        prev[(nx, ny)] = (cx, cy)
                        heappush(heap, (ng + abs(ex - nx) + abs(ey - ny), ng, nx, ny))
        return None

    def _assign_pins(self):
        if not self.taps: return
        p_t_dist = [[abs(p['x']-t['x']) + abs(p['y']-t['y']) for t in self.taps] for p in self.pins]
        loads = [0] * len(self.taps)
        p_order = []
        for pi in range(len(self.pins)):
            ds = sorted((p_t_dist[pi][ti], ti) for ti in range(len(self.taps)))
            regret = (ds[1][0] - ds[0][0]) if len(ds) > 1 else 0
            p_order.append((-regret, pi, ds))
        p_order.sort()
        for _, pi, ds in p_order:
            best_ti, best_s = None, float('inf')
            for d, ti in ds:
                s = d + 0.5 * loads[ti] + (1e12 if self.max_load and loads[ti] >= self.max_load else 0)
                if s < best_s: best_s, best_ti = s, ti
            self.tap_assignments[best_ti].append(pi)
            loads[best_ti] += 1

    def _route_tap(self, ti, deadline):
        pis = self.tap_assignments.get(ti, [])
        if not pis: return True
        connected, remaining = {(self.taps[ti]['x'], self.taps[ti]['y'])}, set(pis)
        while remaining and time.time() < deadline:
            best_p, best_path, best_s = None, None, float('inf')
            ordered = sorted(remaining, key=lambda pi: min(abs(self.pins[pi]['x']-cx)+abs(self.pins[pi]['y']-cy) for cx, cy in connected))
            for pi in ordered[:5]:
                px, py = self.pins[pi]['x'], self.pins[pi]['y']
                sources = sorted(connected, key=lambda p: abs(p[0]-px)+abs(p[1]-py))[:20]
                for sx, sy in sources:
                    path = self._find_path(sx, sy, px, py, ti)
                    if not path: continue
                    s = sum(0.05 if self._ekey(path[i][0], path[i][1], path[i+1][0], path[i+1][1]) in self.tap_edge_sets[ti] else (1.0 + 10.0 * (self.global_edge_usage[self._ekey(path[i][0], path[i][1], path[i+1][0], path[i+1][1])]/self.capacity) + self.history_cost[self._ekey(path[i][0], path[i][1], path[i+1][0], path[i+1][1])]) for i in range(len(path)-1))
                    if s < best_s: best_s, best_p, best_path = s, pi, path
            if not best_path: return False
            for i in range(len(best_path)-1):
                k = self._ekey(best_path[i][0], best_path[i][1], best_path[i+1][0], best_path[i+1][1])
                if k not in self.tap_edge_sets[ti]:
                    self.tap_edge_sets[ti].add(k); self.routing_edges[ti].add(k); self.global_edge_usage[k] += 1
            for node in best_path: connected.add(node)
            remaining.remove(best_p)
        return not remaining

    def _score(self):
        c = sum(max(0, u - self.capacity) for u in self.global_edge_usage.values())
        l = sum(max(0, len(self.tap_assignments.get(ti, [])) - self.max_load) for ti in range(len(self.taps)))
        u = sum(len(self.tap_assignments.get(ti, [])) for ti in range(len(self.taps)) if self.tap_assignments.get(ti) and not self.routing_edges.get(ti))
        return (u, c, l, sum(len(e) for e in self.routing_edges.values()))

    def solve(self):
        if not all([self.grid_size, self.capacity, self.max_load]): return
        deadline = time.time() + self.max_runtime - 0.5
        self._precompute_blocked_edges(); self._assign_pins()
        for ti in sorted(range(len(self.taps)), key=lambda i: len(self.tap_assignments.get(i, [])), reverse=True):
            if time.time() > deadline: break
            self._route_tap(ti, deadline)
        self.best_snapshot = ({k: list(v) for k, v in self.tap_assignments.items()}, {k: set(v) for k, v in self.routing_edges.items()}, {k: set(v) for k, v in self.tap_edge_sets.items()}, dict(self.global_edge_usage))
        bs = self._score()
        for _ in range(15):
            if time.time() > deadline: break
            cong = [k for k, v in self.global_edge_usage.items() if v > self.capacity]
            if not cong and bs[0] == 0: break
            for k in cong: self.history_cost[k] += 2.0
            for ti in sorted(range(len(self.taps)), key=lambda i: sum(self.global_edge_usage[k] for k in self.tap_edge_sets[i]), reverse=True)[:max(1, len(self.taps)//5)]:
                for k in self.tap_edge_sets[ti]: self.global_edge_usage[k] -= 1
                self.tap_edge_sets[ti].clear(); self.routing_edges[ti].clear()
                self._route_tap(ti, deadline)
            cs = self._score()
            if cs < bs: bs = cs; self.best_snapshot = ({k: list(v) for k, v in self.tap_assignments.items()}, {k: set(v) for k, v in self.routing_edges.items()}, {k: set(v) for k, v in self.tap_edge_sets.items()}, dict(self.global_edge_usage))
            else:
                a, r, te, gu = self.best_snapshot
                self.tap_assignments = defaultdict(list, {k: list(v) for k, v in a.items()})
                self.routing_edges = defaultdict(set, {k: set(v) for k, v in r.items()})
                self.tap_edge_sets = defaultdict(set, {k: set(v) for k, v in te.items()})
                self.global_edge_usage = defaultdict(int, gu)

    def write_output(self, out_p):
        lines = []
        for ti in range(len(self.taps)):
            pa, re = self.tap_assignments.get(ti, []), self.routing_edges.get(ti, set())
            lines.append(f'TAP {ti}
PINS {len(pa)}')
            for pi in sorted(pa): lines.append(f'PIN {pi}')
            lines.append(f'ROUTING {len(re)}')
            for e in sorted(re): lines.append(f'EDGE {e[0]} {e[1]} {e[2]} {e[3]}')
        with open(out_p, 'w') as f: f.write('
'.join(lines) + '
')

def parse_input(p):
    try:
        with open(p, 'r') as f: c = f.read().split()
    except: return None
    rt = ml = gs = cp = None
    s = None
    i, n = 0, len(c)
    while i < n:
        tok = c[i]
        if tok in ('MAXRUNTIME', 'MAX_RUNTIME'): rt = int(c[i+1]); i += 2
        elif tok in ('MAXLOAD', 'MAX_LOAD'): ml = int(c[i+1]); i += 2
        elif tok in ('GRIDSIZE', 'GRID_SIZE'): gs = int(c[i+1]); i += 2
        elif tok == 'CAPACITY': cp = int(c[i+1]); i += 2
        elif tok == 'PINS':
            num = int(c[i+1]); i += 2
            if not s: s = CTSSolver(rt, ml, gs, cp)
            for _ in range(num):
                if i+4 <= n and c[i] == 'PIN': s.add_pin(int(c[i+1]), int(c[i+2]), int(c[i+3])); i += 4
                else: i += 1
        elif tok == 'TAPS':
            num = int(c[i+1]); i += 2
            if not s: s = CTSSolver(rt, ml, gs, cp)
            for _ in range(num):
                if i+4 <= n and c[i] == 'TAP': s.add_tap(int(c[i+1]), int(c[i+2]), int(c[i+3])); i += 4
                else: i += 1
        elif tok == 'BLKS':
            num = int(c[i+1]); i += 2
            if not s: s = CTSSolver(rt, ml, gs, cp)
            for _ in range(num):
                if i+6 <= n and c[i] == 'BLK': s.add_blockage(int(c[i+1]), int(c[i+2]), int(c[i+3]), int(c[i+4]), int(c[i+5])); i += 6
                else: i += 1
        else: i += 1
    return s

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', nargs='?'); parser.add_argument('output', nargs='?')
    parser.add_argument('-i', '--input', dest='input_opt'); parser.add_argument('-o', '--output', dest='output_opt')
    args = parser.parse_args()
    inp, out = args.input_opt or args.input, args.output_opt or args.output
    if inp and out:
        s = parse_input(inp)
        if s: s.solve(); s.write_output(out)

if __name__ == '__main__': main()
