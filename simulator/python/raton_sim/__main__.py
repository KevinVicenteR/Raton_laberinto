"""Interfaz de órdenes del simulador."""

import argparse
from pathlib import Path

from raton_sim.giros.native import ROOT


def main(argv=None):
    parser = argparse.ArgumentParser(description="Simulador físico del ratón de laberinto.")
    commands = parser.add_subparsers(dest="command", required=True, title="órdenes")
    descriptions = dict(
        casos="Simula casos extremos y genera SVG e índice.",
        pistas="Simula pistas completas.",
        evaluar="Evalúa controladores.",
        publicar="Genera el catálogo y reproducciones web.",
        pesos="Optimiza los pesos de los cruces.",
        entrenar="Entrena con CMA-ES.",
    )
    for name, description in descriptions.items():
        commands.add_parser(name, help=description, add_help=False)
    args, rest = parser.parse_known_args(argv)
    if args.command == "casos":
        sub = argparse.ArgumentParser(description=descriptions["casos"])
        sub.add_argument(
            "--out", type=Path, default=ROOT / "simulator/results/casos", help="directorio de salida"
        )
        from raton_sim.simulacion.simulator import main as run

        run(sub.parse_args(rest).out)
    elif args.command == "pistas":
        from raton_sim.exploracion.tracks import main as run

        run(rest)
    elif args.command == "evaluar":
        from raton_sim.entrenamiento.evaluation import main as run

        run(rest)
    elif args.command == "publicar":
        from raton_sim.publicacion.publish import main as run

        run(rest)
    elif args.command == "pesos":
        from raton_sim.entrenamiento.weights import main as run

        run(rest)
    else:
        from raton_sim.entrenamiento.train import main as run

        run(rest)


if __name__ == "__main__":
    main()
