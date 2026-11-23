"""
A análise escolhe as execuções pelo motivo do fim (F2.5, R5), sem ROS.

`ftl_analysis.report.find_runs(campanha, reasons=('done',))` devolve, em ordem, as pastas
`<campanha>/<rótulo>/<semente>` cujo `run.yaml` tem um motivo em `reasons`.
"""
from ftl_analysis.report import find_runs
import yaml


def make(root, label, seed, reason):
    folder = root / label / str(seed)
    folder.mkdir(parents=True)
    (folder / 'run.yaml').write_text(yaml.safe_dump({'reason': reason, 'duration': 1.0,
                                                     'seed': seed, 'commits': {}}))
    return folder


def test_find_runs_by_reason(tmp_path):
    done = make(tmp_path, 'a', 1, 'done')
    timeout = make(tmp_path, 'a', 2, 'timeout')
    make(tmp_path, 'b', 1, 'failed')
    assert find_runs(tmp_path) == [done]
    assert find_runs(tmp_path, reasons=('done', 'timeout')) == [done, timeout]
