#!/usr/bin/env python3
import argparse
import time
from collections import defaultdict
from heapq import heappush, heappop

NL = '\n'


class CTSSolver:
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
        self.tap_edge_sets = defaultdict(set)
        self.global_edge_usage = defaultdict(int)
        self.blocked_edges = set()

        self.pin_tap_dist = []
        self.history_cost = defaultdict(float)
        self.best_snapshot = None

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

    @staticmethod
    def _ekey(x1, y1, x2, y2):
        if (x1, y1) <= (x2, y2):
            return (x1, y1, x2, y2)
        return (x2, y2, x1, y1)

    def _precompute_blocked_edges(self):
        self.blocked_edges.clear()
        for b in self.blockages:
            x1, y1, x2, y2 = b['x1'], b['y1'], b['x2'], b['y2']
            for x in range(x1, x2):
                for y in range(y1, y2 + 1):
                    self.blocked_edges.add(self._ekey(x, y, x + 1, y))
            for x in range(x1, x2 + 1):
                for y in range(y1, y2):
                    self.blocked_edges.add(self._ekey(x, y, x, y + 1))

    def _edge_ok_for_tap(self, x1, y1, x2, y2, tap_idx):
        k = self._ekey(x1, y1, x2, y2)
        if k in self.blocked_edges:
            return False
        if k in self.tap_edge_sets[tap_idx]:
            return True
        return self.global_edge_usage[k] < self.capacity

    def _edge_step_cost(self, x1, y1, x2, y2, tap_idx):
        k = self._ekey(x1, y1, x2, y2)
        if k in self.tap_edge_sets[tap_idx]:
            return 0.1
        usage = self.global_edge_usage[k]
        cong = usage / max(1, self.capacity)
        hist = self.history_cost[k]
        return 1.0 + 5.0 * cong + hist

    def _find_path(self, sx, sy, ex, ey, tap_idx):
        if sx == ex and sy == ey:
            return [(sx, sy)]
        gs = self.grid_size
        heap = []
        start_h = abs(ex - sx) + abs(ey - sy)
        heappush(heap, (start_h, 0.0, sx, sy))
        dist = {(sx, sy): 0.0}
        prev = {(sx, sy): None}
        while heap:
            f, g, cx, cy = heappop(heap)
            if (cx, cy) == (ex, ey):
                path = []
                cur = (cx, cy)
                while cur is not None:
                    path.append(cur)
                    cur = prev[cur]
                path.reverse()
                return path
            if g > dist.get((cx, cy), float('inf')):
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < gs and 0 <= ny < gs):
                    continue
                if not self._edge_ok_for_tap(cx, cy, nx, ny, tap_idx):
                    continue
                step = self._edge_step_cost(cx, cy, nx, ny, tap_idx)
                ng = g + step
                if ng < dist.get((nx, ny), float('inf')):
                    dist[(nx, ny)] = ng
                    prev[(nx, ny)] = (cx, cy)
                    h = abs(ex - nx) + abs(ey - ny)
                    heappush(heap, (ng + h, ng, nx, ny))
        return None

    def _precompute_pin_tap_distances(self):
        self.pin_tap_dist = []
        for p in self.pins:
            row = []
            for t in self.taps:
                row.append(abs(p['x'] - t['x']) + abs(p['y'] - t['y']))
            self.pin_tap_dist.append(row)

    def _assign_pins(self):
        self.tap_assignments = defaultdict(list)
        if not self.taps:
            return
        tap_loads = [0] * len(self.taps)
        pin_order = []
        for pi in range(len(self.pins)):
            ds = sorted((self.pin_tap_dist[pi][ti], ti) for ti in range(len(self.taps)))
            best = ds[0][0]
            second = ds[1][0] if len(ds) > 1 else ds[0][0]
            regret = second - best
            pin_order.append((-regret, best, pi, ds))
        pin_order.sort()
        for _, _, pi, ds in pin_order:
            chosen = None
            best_score = float('inf')
            for d, ti in ds:
                overload_penalty = 1e9 if self.max_load is not None and tap_loads[ti] >= self.max_load else 0
                balance_penalty = 0.5 * tap_loads[ti]
                score = d + balance_penalty + overload_penalty
                if score < best_score:
                    best_score = score
                    chosen = ti
            self.tap_assignments[chosen].append(pi)
            tap_loads[chosen] += 1
        self._improve_assignments(tap_loads)

    def _assignment_cost(self, pi, ti, tap_loads):
        d = self.pin_tap_dist[pi][ti]
        overload = 0
        if self.max_load is not None:
            overload = max(0, tap_loads[ti] - self.max_load)
        return d + 1.0 * tap_loads[ti] + 1e7 * overload

    def _improve_assignments(self, tap_loads):
        for _ in range(5):
            moved = False
            for ti in range(len(self.taps)):
                cur_list = list(self.tap_assignments[ti])
                for pi in cur_list:
                    old_cost = self._assignment_cost(pi, ti, tap_loads)
                    best_ti = ti
                    best_gain = 0.0
                    for tj in range(len(self.taps)):
                        if tj == ti:
                            continue
                        tap_loads[ti] -= 1
                        tap_loads[tj] += 1
                        new_cost = self._assignment_cost(pi, tj, tap_loads)
                        gain = old_cost - new_cost
                        tap_loads[ti] += 1
                        tap_loads[tj] -= 1
                        if gain > best_gain:
                            best_gain = gain
                            best_ti = tj
                    if best_ti != ti:
                        self.tap_assignments[ti].remove(pi)
                        self.tap_assignments[best_ti].append(pi)
                        tap_loads[ti] -= 1
                        tap_loads[best_ti] += 1
                        moved = True
            if not moved:
                break

    def _add_path(self, tap_idx, path):
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            k = self._ekey(ax, ay, bx, by)
            if k not in self.tap_edge_sets[tap_idx]:
                self.tap_edge_sets[tap_idx].add(k)
                self.routing_edges[tap_idx].add(k)
                self.global_edge_usage[k] += 1

    def _ripup_tap(self, tap_idx):
        for k in self.tap_edge_sets[tap_idx]:
            self.global_edge_usage[k] -= 1
            if self.global_edge_usage[k] < 0:
                self.global_edge_usage[k] = 0
        self.tap_edge_sets[tap_idx].clear()
        self.routing_edges[tap_idx].clear()

    def _route_tap(self, tap_idx, deadline):
        pis = self.tap_assignments.get(tap_idx, [])
        if not pis:
            return True
        t = self.taps[tap_idx]
        tx, ty = t['x'], t['y']
        connected = {(tx, ty)}
        remaining = set(pis)
        while remaining and time.time() < deadline:
            best_pin = None
            best_path = None
            best_score = float('inf')
            ordered_pins = sorted(
                remaining,
                key=lambda pi: min(abs(self.pins[pi]['x'] - cx) + abs(self.pins[pi]['y'] - cy) for cx, cy in connected)
            )
            for pi in ordered_pins[:5]:
                px, py = self.pins[pi]['x'], self.pins[pi]['y']
                cand_sources = sorted(
                    connected,
                    key=lambda p: abs(p[0] - px) + abs(p[1] - py)
                )[:10]
                for sx, sy in cand_sources:
                    path = self._find_path(sx, sy, px, py, tap_idx)
                    if path is None:
                        continue
                    cost = 0.0
                    for i in range(len(path) - 1):
                        ax, ay = path[i]
                        bx, by = path[i + 1]
                        cost += self._edge_step_cost(ax, ay, bx, by, tap_idx)
                    if cost < best_score:
                        best_score = cost
                        best_pin = pi
                        best_path = path
            if best_path is None:
                return False
            self._add_path(tap_idx, best_path)
            for node in best_path:
                connected.add(node)
            remaining.remove(best_pin)
        return len(remaining) == 0

    def _count_violations(self):
        over_capacity = 0
        for _, usage in self.global_edge_usage.items():
            if self.capacity is not None and usage > self.capacity:
                over_capacity += usage - self.capacity
        load_viol = 0
        if self.max_load is not None:
            for ti in range(len(self.taps)):
                load_viol += max(0, len(self.tap_assignments.get(ti, [])) - self.max_load)
        unrouted = 0
        for ti in range(len(self.taps)):
            assigned = self.tap_assignments.get(ti, [])
            if not assigned:
                continue
            if not self.routing_edges.get(ti):
                unrouted += len(assigned)
        return over_capacity, load_viol, unrouted

    def _total_wirelength(self):
        return sum(len(edges) for edges in self.routing_edges.values())

    def _snapshot(self):
        snap_assign = {k: list(v) for k, v in self.tap_assignments.items()}
        snap_route = {k: set(v) for k, v in self.routing_edges.items()}
        snap_tap_edges = {k: set(v) for k, v in self.tap_edge_sets.items()}
        snap_usage = dict(self.global_edge_usage)
        self.best_snapshot = (snap_assign, snap_route, snap_tap_edges, snap_usage)

    def _restore_snapshot(self):
        if self.best_snapshot is None:
            return
        a, r, te, gu = self.best_snapshot
        self.tap_assignments = defaultdict(list, {k: list(v) for k, v in a.items()})
        self.routing_edges = defaultdict(set, {k: set(v) for k, v in r.items()})
        self.tap_edge_sets = defaultdict(set, {k: set(v) for k, v in te.items()})
        self.global_edge_usage = defaultdict(int, gu)

    def _score_tuple(self):
        over_capacity, load_viol, unrouted = self._count_violations()
        wl = self._total_wirelength()
        return (unrouted, over_capacity, load_viol, wl)

    def solve(self):
        if self.grid_size is None or self.capacity is None or self.max_load is None:
            return
        t0 = time.time()
        # Use almost the full runtime budget, while keeping a small safety margin so we never exceed MAXRUNTIME.
        deadline = t0 + max(0.1, self.max_runtime - 0.1)
        self._precompute_blocked_edges()
        self._precompute_pin_tap_distances()
        self._assign_pins()

        tap_order = sorted(range(len(self.taps)), key=lambda ti: len(self.tap_assignments.get(ti, [])), reverse=True)
        for ti in tap_order:
            if time.time() > deadline:
                break
            self._route_tap(ti, deadline)

        self._snapshot()
        best_score = self._score_tuple()

        rounds = 0
        while time.time() < deadline and rounds < 10:
            rounds += 1
            congested_edges = [k for k, v in self.global_edge_usage.items() if self.capacity is not None and v > self.capacity]
            if not congested_edges and best_score[0] == 0 and best_score[1] == 0 and best_score[2] == 0:
                break
            for k in congested_edges:
                self.history_cost[k] += 1.0

            reroute_list = sorted(range(len(self.taps)), key=lambda ti: sum(self.global_edge_usage[k] for k in self.tap_edge_sets[ti]), reverse=True)
            for ti in reroute_list[:max(1, len(self.taps) // 4 or 1)]:
                if time.time() > deadline:
                    break
                self._ripup_tap(ti)
                self._route_tap(ti, deadline)

            cur_score = self._score_tuple()
            if cur_score < best_score:
                best_score = cur_score
                self._snapshot()
            else:
                self._restore_snapshot()

    def write_output(self, output_file):
        lines = []
        for ti in range(len(self.taps)):
            pin_indices = self.tap_assignments.get(ti, [])
            edges = self.routing_edges.get(ti, set())
            lines.append(f'TAP {ti}')
            lines.append(f'PINS {len(pin_indices)}')
            for pi in pin_indices:
                lines.append(f'PIN {pi}')
            lines.append(f'ROUTING {len(edges)}')
            for (x1, y1, x2, y2) in edges:
                lines.append(f'EDGE {x1} {y1} {x2} {y2}')
        with open(output_file, 'w') as f:
            f.write(NL.join(lines) + NL)


def parse_input(input_file):
    try:
        with open(input_file, 'r') as f:
            content = f.read().split()
    except Exception:
        return None

    rt = ml = gs = cp = None
    solver = None
    i = 0
    n = len(content)
    while i < n:
        tok = content[i]
        if tok in ('MAXRUNTIME', 'MAX_RUNTIME') and i + 1 < n:
            rt = int(content[i + 1])
            i += 2
        elif tok in ('MAXLOAD', 'MAX_LOAD') and i + 1 < n:
            ml = int(content[i + 1])
            i += 2
        elif tok in ('GRIDSIZE', 'GRID_SIZE') and i + 1 < n:
            gs = int(content[i + 1])
            i += 2
        elif tok == 'CAPACITY' and i + 1 < n:
            cp = int(content[i + 1])
            i += 2
        elif tok == 'PINS' and i + 1 < n:
            num = int(content[i + 1])
            i += 2
            if solver is None:
                solver = CTSSolver(rt, ml, gs, cp)
            for _ in range(num):
                if i + 4 <= n and content[i] == 'PIN':
                    solver.add_pin(int(content[i + 1]), int(content[i + 2]), int(content[i + 3]))
                    i += 4
        elif tok == 'TAPS' and i + 1 < n:
            num = int(content[i + 1])
            i += 2
            for _ in range(num):
                if i + 4 <= n and content[i] == 'TAP':
                    solver.add_tap(int(content[i + 1]), int(content[i + 2]), int(content[i + 3]))
                    i += 4
        elif tok == 'BLKS' and i + 1 < n:
            num = int(content[i + 1])
            i += 2
            for _ in range(num):
                if i + 6 <= n and content[i] == 'BLK':
                    solver.add_blockage(
                        int(content[i + 1]),
                        int(content[i + 2]),
                        int(content[i + 3]),
                        int(content[i + 4]),
                        int(content[i + 5]),
                    )
                    i += 6
        else:
            i += 1
    return solver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', nargs='?', help='Input file path')
    parser.add_argument('output_file', nargs='?', help='Output file path')
    parser.add_argument('-i', '--input', dest='input_opt', help='Input file path')
    parser.add_argument('-o', '--output', dest='output_opt', help='Output file path')
    args = parser.parse_args()

    input_path = args.input_opt or args.input_file
    output_path = args.output_opt or args.output_file

    if input_path is None or output_path is None:
        parser.error('Both input and output file paths must be specified.')

    solver = parse_input(input_path)
    if solver:
        solver.solve()
        solver.write_output(output_path)


if __name__ == '__main__':
    main()
