import pytest

from py_ballisticcalc_exts.base_traj_seq import CBaseTrajSeq


def test_reserve_and_append_growth():
    seq = CBaseTrajSeq()
    seq.reserve(1)
    for i in range(8):
        seq.append(float(i), i, i, i, i, i, i, 0.5 + 0.01 * i)
    assert len(seq) == 8
    # Spot check first and last
    assert seq[0].time == pytest.approx(0.0)
    assert seq[-1].time == pytest.approx(7.0)


def test_interpolate_bounds_checks():
    seq = CBaseTrajSeq()
    # Need at least three points and middle index is required
    for i in range(3):
        seq.append(float(i), 10.0 * i, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6)

    # Valid case: idx=1 has neighbors on both sides
    mid = seq.interpolate_at(1, "time", 0.5)
    assert mid.time == pytest.approx(0.5)

    # Error cases: out-of-range or no neighbors
    with pytest.raises(IndexError):
        seq.interpolate_at(0, "time", 0.5)
    with pytest.raises(IndexError):
        seq.interpolate_at(2, "time", 1.5)
