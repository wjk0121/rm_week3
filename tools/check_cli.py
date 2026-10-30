"""Integration checks for timestamp sources, bounds and cached/raw detector equivalence."""
import csv
import json
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'build/rm_week3'
CACHE = ROOT / 'results/revision/nn_detections.cache'

def run(*args, ok=True):
    result = subprocess.run([str(APP), *map(str, args)], cwd=ROOT, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
    if (result.returncode == 0) != ok:
        raise AssertionError(result.stdout)
    return result.stdout

def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))

def main():
    if not CACHE.exists():
        raise SystemExit('Generate the complete NN detection cache before integration checks')
    with tempfile.TemporaryDirectory(prefix='rm-week3-cli-') as directory:
        temp = Path(directory)
        run('--no-video', '--plate-radius', '.5', ok=False)
        run('--no-video', '--imu-offset', 'nan', ok=False)
        run('--time-mode', 'unknown', ok=False)
        run('--time-mode', 'video', ok=False)
        raw, cached = temp / 'raw.csv', temp / 'cached.csv'
        run('--no-video', '--max-frames', 8, '--csv', raw)
        run('--no-video', '--max-frames', 8, '--detection-cache', CACHE, '--csv', cached)
        a, b = rows(raw), rows(cached)
        for ra, rb in zip(a, b):
            for key in ('cx', 'cy', 'za', 'theta', 'omega', 'r', 'dr', 'dh'):
                if abs(float(ra[key])-float(rb[key])) > 1e-6:
                    raise AssertionError(f'Cache changed estimates: {key}')
        imu_times = [float(line.split()[0]) for line in (ROOT/'data/imu.txt').read_text().splitlines() if line.strip()]
        timestamps = temp/'times.csv'
        # Synthetic integration fixture, not a claim about the recording's real capture times.
        timestamps.write_text('frame,t_image_s\n'+''.join(f'{i},{t+.009:.12f}\n' for i,t in enumerate(imu_times)))
        explicit = temp/'explicit.csv'
        run('--no-video', '--frame-times', timestamps, '--max-frames', 8,
            '--detection-cache', CACHE, '--csv', explicit)
        meta = json.loads(Path(str(explicit)+'.meta.json').read_text())
        assert meta['time_mode'] == 'timestamps' and meta['imu_clamped_frames'] == 0
        assert meta['time_correspondence_verified'] == 0
        timestamps.write_text('frame,t_image_s\n'+''.join(f'{i},{t-1:.12f}\n' for i,t in enumerate(imu_times)))
        run('--no-video', '--frame-times', timestamps, '--max-frames', 8,
            '--detection-cache', CACHE, '--csv', temp/'outside.csv', ok=False)
    print('PASS: strict CLI, mechanical bounds, explicit timestamps, IMU coverage, cached/raw equivalence')

if __name__ == '__main__':
    main()
