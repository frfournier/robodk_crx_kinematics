"""External artifact comparisons allow improved coverage and expose regressions."""

from pathlib import Path
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import benchmark_solvers as benchmark


def test_comparison_reports_coverage_and_validates_each_artifact(tmp_path, monkeypatch):
    robot = np.zeros((93, 20))
    robot[30, :6], robot[31, :6] = -180, 180
    models = [{"name": "model", "robot": robot}]
    cases = [dict(name=f"case-{index}", model=0, target=np.eye(4).tolist(),
                  group="random", reachable=True, offset_rad=0,
                  witnesses=[[0.] * 6]) for index in range(3)]

    def independent_fk(reference, commands):
        poses = np.tile(np.eye(4), (len(commands), 1, 1))
        poses[:, 0, 3] = commands[:, 0]
        return poses

    monkeypatch.setattr(benchmark, "reference_poses", independent_fk)
    timings, solutions = [], []
    # Extra valid candidates, a gained solve, and an inaccurate candidate return.
    for index, counts in enumerate(((1, 2), (0, 1), (1, 1))):
        for label, count in zip(benchmark.PATHS, counts):
            for mode in benchmark.MODES:
                timings.append(dict(case=index, mode=mode, path=label, round=0,
                                    us=10., count=count))
                for solution in range(-1, count) if count else ():
                    command = [0.] * 6
                    if index == 2 and label == "candidate":
                        command[0] = 1.
                    solutions.append(dict(case=index, mode=mode, path=label,
                                          solution=solution,
                                          **{f"q{i+1}": q for i, q in enumerate(command)}))
    pd.DataFrame(timings).to_csv(tmp_path / "timings.csv", index=False)
    pd.DataFrame(solutions).to_csv(tmp_path / "solutions.csv", index=False)

    _, errors, frame = benchmark.validate(tmp_path, models, [None], cases, 1)
    comparison = pd.read_csv(tmp_path / "comparison.csv").set_index(["case", "mode"])
    for mode in benchmark.MODES:
        assert not comparison.loc[(0, mode), "lost_solve"]
        assert not comparison.loc[(0, mode), "gained_solve"]
        assert comparison.loc[(0, mode), "count_candidate"] == 2
        assert comparison.loc[(1, mode), "gained_solve"]
        assert comparison.loc[(2, mode), "lost_solve"]
        assert not comparison.loc[(2, mode), "gained_solve"]
        assert comparison.loc[(2, mode), "lost_witnesses"]
    assert errors[errors.path == "baseline"].valid.all()
    assert not errors[(errors.path == "candidate") & (errors["case"] == 2)].valid.any()
    assert frame[frame.path == "candidate"].invalid_solutions.sum() == 2
