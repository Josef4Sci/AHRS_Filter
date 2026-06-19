import matplotlib.pyplot as plt


base_fold = 'paper_j2\\figures\\'
# Algorithm: (time_ns, rmse_white, rmse_disturb)
data = {
    'Madgwick': (71.4,    2.87,    4.24),
    'Justa': (64.91,  2.84,  3.96),
    'Basic VQF': (165.01,  2.71,   2.42),
    'VQF':      (317.52,   2.66,    1.98),
    'Fast VQF\n (ours)': (87.03,  2.62,  2.92),
}

origin = 'Fast VQF\n (ours)'
origin_time = data[origin][0]
for key in data:
    t, rmse_w, rmse_d = data[key]
    rel_time = t / origin_time
    print(f"{key}: {rel_time * 100:.2f}%")


fig, ax = plt.subplots(figsize=(5, 4))

top_side = ['VQF', 'Basic VQF']
transparent = {'Madgwick'}

default_color_sequence = plt.rcParams['axes.prop_cycle'].by_key()['color']
#swap colors to match paper figure
color_sequence = [default_color_sequence[2], default_color_sequence[1], default_color_sequence[4], default_color_sequence[0], default_color_sequence[3]]
#colors = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4']  # red, green, blue, orange, purple
markers = ['o', 's', '^', 'D', 'P']  # circle, square, triangle, diamond, plus

for (label, (t, rmse_w, rmse_d)), color, marker in zip(data.items(), color_sequence, markers):
    alpha = 0.35 if label in transparent else 1.0
    ax.scatter(t, rmse_w, color=color, marker=marker, s=80, zorder=5, alpha=alpha, label=f'{label} (clean)')
    ax.scatter(t, rmse_d, color=color, marker=marker, s=80, zorder=5,
               facecolors='none', linewidths=1.8, alpha=alpha, label=f'{label} (disturb)')
    # vertical line connecting clean and disturb for same algorithm
    ax.plot([t, t], [rmse_w, rmse_d], color=color, lw=1, ls='--', alpha=0.5 * alpha)
    # label next to point
    
    mult = 0.8 if label in top_side else -1.0
    off_side = 0
    off_top = 0
    if label in ['Fast VQF\n (ours)']:
        off_side += 0
        off_top = -14
    if label in ['Madgwick']:
        off_side = 24
        off_top = 34
    ax.annotate(label, xy=(t, rmse_w), xytext=(off_side, off_top+int(mult*14)), textcoords='offset points',
                fontsize=9, color=color, ha='center', alpha=alpha)

ax.set_xlabel('Computation time (ns/sample)')
ax.set_ylabel('RMSE (°)')
ax.set_title('Algorithm speed vs. accuracy trade-off')

# custom legend: filled = clean noise, open = all noise
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker='o', color='k', markerfacecolor='k', markersize=7, lw=0, label='Undisturbed (White list dataset)'),
    Line2D([0], [0], marker='o', color='k', markerfacecolor='none', markersize=7,
           markeredgewidth=1.8, lw=0, label='Disturb takes (fixed ref)'),
]
ax.legend(handles=legend_elements, fontsize=8)

ax.grid(True, alpha=0.3)
ax.set_xlim(left=0, right=350)
ax.set_ylim(bottom=1.0, top=5.0)

plt.tight_layout()
# plt.savefig(base_fold + 'algo_time_vs_accuracy.pdf', bbox_inches='tight')
plt.savefig(base_fold + 'algo_time_vs_accuracy.png', dpi=300, bbox_inches='tight')
plt.show()
