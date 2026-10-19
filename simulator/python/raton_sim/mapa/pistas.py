import random

from raton_sim.giros import native as N

M_L = [  # sube 3 celdas y gira a la izquierda hasta la meta (0,2)
    "+---+---+---+",
    "|           |",
    "+---+---+   +",
    "|       |   |",
    "+---+---+   +",
    "|       |   |",
    "+---+---+---+",
]
M_T = [  # sube y llega a una T: a la derecha callejon, a la izquierda la meta
    "+---+---+---+---+---+",
    "|                   |",
    "+---+---+   +---+---+",
    "|       |   |       |",
    "+---+---+   +---+---+",
    "|       |   |       |",
    "+---+---+---+---+---+",
]
M_DEAD = [  # cruce: a la derecha callejon sin salida, la meta arriba a la izquierda
    "+---+---+---+",
    "|   |   |   |",
    "+   +---+---+",
    "|           |",
    "+---+   +---+",
    "|   |   |   |",
    "+---+---+---+",
]


def random_maze(cols=19, rows=10, seed=0, braid=0.12, start=(18, 0)):
    """Laberinto aleatorio (DFS) con un porcentaje de paredes quitadas para crear bucles."""
    rnd = random.Random(seed)
    H = [[True] * cols for _ in range(rows + 1)]  # H[j][i]: pared horizontal en la linea j (desde arriba)
    V = [[True] * rows for _ in range(cols + 1)]  # V[i][j]: pared vertical en la linea i (desde la izquierda)
    seen = {start}
    stack = [start]
    while stack:
        x, y = stack[-1]
        nb = [
            (x + dx, y + dy)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= x + dx < cols and 0 <= y + dy < rows and (x + dx, y + dy) not in seen
        ]
        if not nb:
            stack.pop()
            continue
        nx, ny = rnd.choice(nb)
        r, nr = rows - 1 - y, rows - 1 - ny
        if nx != x:
            V[max(x, nx)][r] = False
        else:
            H[max(r, nr)][x] = False
        seen.add((nx, ny))
        stack.append((nx, ny))
    for j in range(1, rows):
        for i in range(cols):
            if H[j][i] and rnd.random() < braid:
                H[j][i] = False
    # Como en la competencia, la salida mira a un pasillo abierto (al norte).
    sx, sy = start
    if sy + 1 < rows:
        H[rows - 1 - sy][sx] = False
    for i in range(1, cols):
        for j in range(rows):
            if V[i][j] and rnd.random() < braid:
                V[i][j] = False
    lines = []
    for j in range(rows + 1):
        lines.append("+" + "+".join("---" if H[j][i] else "   " for i in range(cols)) + "+")
        if j < rows:
            lines.append(
                "".join(("|" if V[i][j] else " ") + "   " for i in range(cols)) + ("|" if V[cols][j] else " ")
            )
    return lines


def comp_maze():
    with open(N.ROOT / "firmware/mapa/competition_maze.h") as source:
        src = source.read()
    return [
        l.split('"')[1] for l in src.splitlines() if l.strip().startswith('"+') or l.strip().startswith('"|')
    ]
