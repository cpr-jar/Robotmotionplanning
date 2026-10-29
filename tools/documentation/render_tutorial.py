"""Regenerate beginner flowcharts (Graphviz) and the numerical timing illustration.

Uses optional matplotlib from tools/requirements-optional.txt for the teaching plot.
Does not import or modify the robot motion implementation.
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
DIAGRAMS = ROOT / 'docs/tutorials/diagrams'


def main():
    for source in sorted(DIAGRAMS.glob('*.dot')):
        subprocess.run(['dot', '-Tsvg', str(source), '-o', str(source.with_suffix('.svg'))], check=True)
    import matplotlib
    matplotlib.use('Agg')
    matplotlib.rcParams.update({'svg.fonttype': 'none', 'svg.hashsalt': 'motion-planning-basics-v1'})
    import matplotlib.pyplot as plt
    duration_s = 4
    times = [index * duration_s / 400 for index in range(401)]
    progress = [(time / duration_s) ** 3 * (10 - 15 * time / duration_s + 6 * (time / duration_s) ** 2) for time in times]
    velocities = [30 * (time / duration_s) ** 2 * (1 - time / duration_s) ** 2 / duration_s for time in times]
    assert progress[0] == 0 and progress[-1] == 1
    assert abs(progress[100] - .103515625) < 1e-12
    assert abs(max(velocities) - .46875) < 1e-12
    figure, axes = plt.subplots(2, 1, figsize=(8.2, 4.8), sharex=True, layout='constrained')
    axes[0].plot(times, progress, color='#2065a7', linewidth=2.4)
    axes[0].set_ylabel('Position q [rad]')
    axes[0].set_ylim(-.05, 1.05)
    axes[0].set_yticks([0, .5, 1])
    axes[1].plot(times, velocities, color='#087e8b', linewidth=2.4, label='Geplanter Verlauf')
    axes[1].axhline(.5, color='#b45555', linestyle='--', label='Geschwindigkeitsgrenze 0,5 rad/s')
    axes[1].set_ylabel('Geschwindigkeit [rad/s]')
    axes[1].set_ylim(-.03, .56)
    axes[1].set_xlabel('Zeit t [s]')
    axes[1].legend(loc='upper right', fontsize=8)
    for axis in axes:
        axis.grid(alpha=.2)
        axis.set_xlim(0, duration_s)
        axis.set_xticks([0, 1, 2, 3, 4])
    figure.suptitle('Lehrbeispiel: eine Achse von 0 auf 1 rad in 4 s')
    figure.savefig(DIAGRAMS / '06_quintic_timing.svg', metadata={'Date': None, 'Creator': 'Robot Motion tutorial'})
    plt.close(figure)
    for output in DIAGRAMS.glob('*.svg'):
        output.write_text('\n'.join(line.rstrip() for line in output.read_text().splitlines()) + '\n')
    print('Rendered five beginner flowcharts and one verified timing illustration')


if __name__ == '__main__':
    main()
