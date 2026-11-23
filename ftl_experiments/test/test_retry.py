"""
A repetição das execuções que falham (F2.5, R7), sem ROS.

`ftl_experiments.campaign.retry(attempt, retries)` chama `attempt()` (que devolve o motivo do fim)
até o motivo não ser `failed` ou até `retries` repetições, e devolve `(motivo, tentativas)`.
`write_run(..., attempts=n)` grava `attempts` no `run.yaml`; sem ele, o campo não aparece.
"""
from ftl_experiments.campaign import read_run, retry, write_run


def sequence(*reasons):
    calls = iter(reasons)
    return lambda: next(calls)


def test_retry():
    assert retry(sequence('done'), 1) == ('done', 1)
    assert retry(sequence('failed', 'timeout'), 1) == ('timeout', 2)
    assert retry(sequence('failed', 'failed', 'done'), 1) == ('failed', 2)
    assert retry(sequence('failed', 'failed', 'done'), 2) == ('done', 3)
    assert retry(sequence('failed'), 0) == ('failed', 1)


def test_attempts_in_the_run_file(tmp_path):
    write_run(tmp_path, 'done', 10.0, 1, {}, attempts=2)
    assert read_run(tmp_path)['attempts'] == 2
    write_run(tmp_path, 'done', 10.0, 1, {})
    assert 'attempts' not in read_run(tmp_path)
