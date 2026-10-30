"""Plot all available vehicle states with 2-sigma bands and reset boundaries."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FIELDS = [('cx', 'center x [m]'), ('vx', 'vx [m/s]'),
          ('cy', 'center y [m]'), ('vy', 'vy [m/s]'),
          ('za', 'base plate height [m]'), ('vz', 'vz [m/s]'),
          ('theta', 'yaw [deg], unwrap per track'), ('omega', 'omega [rad/s]'),
          ('r', 'even radius [m]'), ('dr', 'odd - even radius [m]'),
          ('dh', 'odd - even height [m]')]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('output', type=Path, nargs='?')
    args = parser.parse_args()
    data = np.atleast_1d(np.genfromtxt(args.csv, delimiter=',', names=True))
    fields = [(i, f, title) for i, (f, title) in enumerate(FIELDS) if f in data.dtype.names]
    reset = np.zeros(len(data), dtype=bool)
    for name in ('initialization', 'reinit'):
        if name in data.dtype.names:
            reset |= np.nan_to_num(data[name]) > 0
    valid = np.isfinite(data['cx'])
    reset |= valid & np.r_[True, ~valid[:-1]]
    count = (len(fields) + 1) // 2
    fig, axes = plt.subplots(count, 2, figsize=(13, 2.3 * count), sharex=True, squeeze=False)
    for ax, (index, name, title) in zip(axes.flat, fields):
        values = data[name].astype(float).copy()
        if name == 'theta':
            begin = None
            for n in range(len(values) + 1):
                boundary = n == len(values) or not np.isfinite(values[n]) or reset[n]
                if boundary and begin is not None:
                    values[begin:n] = np.degrees(np.unwrap(values[begin:n]))
                    begin = None
                if n < len(values) and np.isfinite(values[n]) and begin is None:
                    begin = n
        # Do not visually connect states across a lost track / new reference slot.
        boundaries = reset.copy()
        if len(boundaries):
            boundaries[0] = False
        values[boundaries] = np.nan
        ax.plot(data['t'], values, linewidth=1, label='estimate')
        covariance = f'P{index}'
        if covariance in data.dtype.names:
            sigma = np.sqrt(np.maximum(data[covariance], 0))
            if name == 'theta':
                sigma = np.degrees(sigma)
            ax.fill_between(data['t'], values - 2*sigma, values + 2*sigma,
                            color='tab:blue', alpha=.15, label='±2 sigma (model)')
        for stamp in data['t'][boundaries]:
            ax.axvline(stamp, color='tab:red', linewidth=.6, alpha=.5)
        ax.set_ylabel(title, fontsize=9)
        ax.grid(alpha=.25)
    for ax in list(axes.flat)[len(fields):]:
        ax.set_visible(False)
    for ax in axes[-1]:
        ax.set_xlabel('time [s]')
    axes.flat[0].legend(fontsize=8)
    fig.suptitle(f'{len(fields)}-state vehicle estimate; uncertainty is conditional on model assumptions')
    fig.tight_layout()
    output = args.output or args.csv.with_suffix('.png')
    fig.savefig(output, dpi=140)
    print(output)

if __name__ == '__main__':
    main()
