"""Summarize frame coverage and model residuals; no ground-truth accuracy is implied."""
import argparse
import csv
import math
import statistics
from pathlib import Path

def finite(rows, field):
    result = []
    for row in rows:
        try:
            value = float(row.get(field, 'nan'))
        except (ValueError, TypeError):
            continue
        if math.isfinite(value):
            result.append(value)
    return result

def summarize(path, warmup):
    with path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'Empty CSV: {path}')
    settled = [r for r in rows if int(r['frame']) >= warmup]
    matched = [r for r in settled if int(r['n_matched']) > 0]
    initialized = sum(int(r.get('initialization', r['reinit'])) > 0 for r in rows)
    reinitializations = sum(int(r['reinit']) > 0 for r in rows) if 'initialization' in rows[0] else max(0, initialized-1)
    print(f'{path}: frames={len(rows)}, matched after warmup={len(matched)}/{len(settled)}, '
          f'initializations={initialized}, reinitializations={reinitializations}')
    for name in ('pre_reproj_rms_px', 'pre_reproj_all_rms_px', 'post_reproj_all_rms_px'):
        values = sorted(finite(matched, name))
        if values:
            print(f'  {name}: median={statistics.median(values):.3f} p90={values[int(.9*(len(values)-1))]:.3f}')
    tail = [r for r in rows if float(r['t']) >= float(rows[-1]['t'])-5]
    for name in ('r', 'dr', 'dh'):
        values = finite(tail, name)
        if values:
            print(f'  last5s {name}: median={statistics.median(values):.5f} range={max(values)-min(values):.5f} m')
    if 'covariance_ok' in rows[0]:
        valid = [r for r in rows if finite([r], 'cx')]
        print(f"  covariance SPD={sum(int(r['covariance_ok']) for r in valid)}/{len(valid)}, "
              f"IMU boundary holds={sum(int(r['imu_clamped']) for r in rows)}")
    normalized = [float(r['nis'])/int(r['nis_dof']) for r in matched
                  if int(r.get('nis_dof', 0)) > 0 and finite([r], 'nis')]
    if normalized:
        print(f'  accepted-update mean NIS/dof={statistics.mean(normalized):.3f} '
              '(selected/gated sample; not an unbiased calibration test)')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', type=Path)
    parser.add_argument('--warmup-frames', type=int, default=30)
    args = parser.parse_args()
    for path in args.csv:
        summarize(path, args.warmup_frames)

if __name__ == '__main__':
    main()
