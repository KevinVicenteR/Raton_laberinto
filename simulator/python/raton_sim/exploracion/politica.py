from raton_sim.entrenamiento.politica import Gate


class MissionOrchestrator:
    def __init__(self, gate=None):
        self.gate = Gate(gate)
        self.failures = 0
        self.pruned = set()
        self.unreachable = set()
        self.state = "read_start"

    def decide(self, start=False, finished=False, route=False, recover=False):
        # Safety and mission completion cannot be overridden by learned weights.
        self.state = (
            "finish"
            if finished
            else (
                "read_start"
                if start
                else ("recover" if recover else ("shortest_route" if route else "explore_step"))
            )
        )
        self.gate.weights_for([self.failures], 1)
        return self.state

    def choose(self, ordered_options):
        weights = self.gate.weights_for([self.failures], len(ordered_options))
        return min(enumerate(ordered_options), key=lambda pair: (pair[0] + 1) / weights[pair[0]])[1]

    def attempts(self, limit):
        return range(max(1, int(limit)))

    def progress(self, moved, node, limit):
        self.failures = 0 if moved else self.failures + 1
        if self.failures >= limit:
            self.unreachable.add(node)
            return False
        return True

    def prune(self, seen, goal, pos, wall, moves):
        changed = True
        while changed:
            changed = False
            for node in seen - self.pruned - {goal, pos}:
                exits = [
                    (node[0] + dx, node[1] + dy) for k, (dx, dy) in enumerate(moves) if not wall(*node, k)
                ]
                if sum(n not in self.pruned for n in exits) <= 1:
                    self.pruned.add(node)
                    changed = True
        return self.pruned
