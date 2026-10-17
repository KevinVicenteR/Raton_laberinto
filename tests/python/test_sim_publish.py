"""Contrato del catálogo que consume la web, incluidos sus archivos reales."""

import json
from pathlib import Path

from raton_sim.publish import publication_jobs, replay_description, subsample

ROOT = Path(__file__).resolve().parents[2]


def test_publication_matrix():
    jobs = publication_jobs("mapped")
    assert len(jobs) == 20
    assert len(set(jobs)) == 20
    for suite, seed, mode in (("competition", 0, 4), ("random", 100, 3), ("random", 101, 3)):
        assert {j[6] for j in jobs if (j[4], j[0], j[2]) == (suite, seed, mode)} == {140, 168, 192, 220}
    assert len({j[0] for j in jobs if j[4] == "random" and j[2] == 3}) >= 6
    assert len({j[0] for j in jobs if j[4] == "random" and j[2] == 5}) >= 2
    assert (0, 1, 3, "mapped", "competition", True, 192) in jobs
    assert (0, 1, 4, "mapped", "altered", True, 192) in jobs


def test_subsample_preserves_endpoints():
    samples = list(range(10000))
    reduced = subsample(samples, 1800)
    assert reduced[0] == 0 and reduced[-1] == 9999
    assert len(reduced) <= 1800
    assert reduced == sorted(set(reduced))
    assert subsample([1], 1800) == [1]


def test_published_catalog():
    folder = ROOT / "web/public/sim"
    catalog = json.loads((folder / "dashboard.json").read_text())
    assert catalog["generated_at"] and catalog["run_id"]
    assert catalog["default_controller"] in catalog["controller_labels"]
    assert len(catalog["results"]) == len(catalog["replays"]) >= 20
    assert len({r["id"] for r in catalog["replays"]}) == len(catalog["replays"])
    for row, replay in zip(catalog["results"], catalog["replays"], strict=True):
        path = folder / Path(replay["file"]).name
        assert replay["file"] == "sim/" + path.name
        assert path.stat().st_size < 1_500_000
        data = json.loads(path.read_text())
        for key in ("mode", "contacts", "clean_arrival", "reached", "corridor_mm", "failure", "elapsed"):
            assert data[key] == row[key]
        assert data["trace"]
        assert data["pitch"] == row["corridor_mm"] + 12
        assert f"{row['corridor_mm']} mm" in replay["label"]
        assert (replay["label"], replay["summary"]) == replay_description(row, row["maze"])
        if row["failure"]:
            assert row["failure"] in replay["label"]
    assert sum(p.stat().st_size for p in folder.iterdir() if p.is_file()) < 30_000_000
