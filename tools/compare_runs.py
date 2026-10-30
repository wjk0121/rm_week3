"""Create a comparison on identical frames, and check objective regression gates."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path

def read(path):
    with path.open(encoding='utf-8', newline='') as stream:
        return {int(r['frame']): r for r in csv.DictReader(stream)}

def summary(values):
    values = sorted(values)
    return {'n': len(values), 'median': statistics.median(values),
            'p90': values[int(.9*(len(values)-1))]}

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('baseline', type=Path)
    p.add_argument('updated', type=Path)
    p.add_argument('baseline_reprojection', type=Path)
    p.add_argument('updated_reprojection', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    old, new = read(a.baseline), read(a.updated)
    ro, rn = read(a.baseline_reprojection), read(a.updated_reprojection)
    if set(old) != set(new) or set(ro) != set(rn) or set(new) != set(rn):
        raise ValueError('Frame coverage differs; compare identical frames first')
    evaluation = [f for f in sorted(new) if f >= 30]
    for f in evaluation:
        if ro[f]['n_detected'] != rn[f]['n_detected'] or ro[f]['n_evaluated'] != rn[f]['n_evaluated']:
            raise ValueError('Raw detection evaluation coverage differs')
    def finite(rows, field, frames):
        return [float(rows[f][field]) for f in frames if math.isfinite(float(rows[f][field]))]
    common_matched = [f for f in evaluation if int(old[f]['n_matched']) and int(new[f]['n_matched'])]
    result = {'evaluated_frames': len(evaluation), 'common_matched_frames': len(common_matched)}
    for name, states, reproj in [('baseline', old, ro), ('updated', new, rn)]:
        result[name] = {'matched_frames': sum(int(states[f]['n_matched']) > 0 for f in evaluation),
                        'pre_reprojection_matched': summary(finite(states, 'pre_reproj_rms_px', evaluation)),
                        'pre_reprojection_common_frames': summary(finite(states, 'pre_reproj_rms_px', common_matched)),
                        'all_detection_reprojection': summary(finite(reproj, 'rms_px', evaluation))}
    old_med = result['baseline']['all_detection_reprojection']['median']
    new_med = result['updated']['all_detection_reprojection']['median']
    result['median_reduction_percent'] = 100*(old_med-new_med)/old_med
    checks = {'all_frames_processed': len(new) == 531,
              'no_pose_rejection': all(r['n_detected'] == r['n_meas'] for r in new.values()),
              'covariance_positive_definite': all(int(r['covariance_ok']) for r in new.values()),
              'no_reinitialization': not any(int(r['reinit']) for r in new.values()),
              'matched_coverage_not_lower': result['updated']['matched_frames'] >= result['baseline']['matched_frames'],
              'raw_reprojection_median_improved': new_med < old_med,
              'raw_reprojection_p90_improved': result['updated']['all_detection_reprojection']['p90'] < result['baseline']['all_detection_reprojection']['p90']}
    result['checks'] = checks
    a.output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if not all(checks.values()):
        raise SystemExit('At least one real-video regression gate failed')

if __name__ == '__main__':
    main()
