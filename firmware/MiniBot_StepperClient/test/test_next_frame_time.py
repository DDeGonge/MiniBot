frame_len_us = 100 * 1000  # 100000 us


def calc_next(sync, now):
    return sync + ((now - sync) // frame_len_us + 1) * frame_len_us


cases = [
    ("sync=0, mid first frame", 0, 50000),
    ("sync=0, just before boundary", 0, 99999),
    ("sync=0, exactly on boundary", 0, 100000),
    ("sync=0, just after boundary", 0, 100001),
    ("sync=0, deep into run", 0, 550000),
    ("sync=12345, mid frame", 12345, 150000),
    ("sync=12345, on boundary", 12345, 112345),
    ("sync=12345, before boundary", 12345, 112344),
    ("sync=12345, after boundary", 12345, 112346),
    ("large time, mid frame", 10**9, 10**9 + 50000),
    ("large time, on boundary", 10**9, 10**9 + 100000),
    ("now == sync", 500000, 500000),
    ("now == sync+1", 500000, 500001),
]

passed = 0
for name, sync, now in cases:
    r = calc_next(sync, now)
    gt = r > now
    aligned = (r - sync) % frame_len_us == 0
    gt_s = "OK" if gt else "FAIL"
    al_s = "OK" if aligned else "FAIL"
    print(
        f"{name:40s} sync={sync:>12d} now={now:>12d} next={r:>12d}  >now:{gt_s}  aligned:{al_s}"
    )
    assert gt, f"FAIL: next ({r}) <= now ({now})"
    assert aligned, f"FAIL: (next-sync) % frame_len != 0"
    passed += 1

print(f"\nAll {passed} tests passed.")
