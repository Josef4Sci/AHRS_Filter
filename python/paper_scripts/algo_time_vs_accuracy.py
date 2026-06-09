import matplotlib.pyplot as plt


base_fold = 'paper_j2\\figures\\'
# Algorithm: (time_ns, rmse_white, rmse_all)
data = {
    'Fast VQF (ours)': (7.750,  2.6775,  3.293874),
    'Base VQF': (15.267,  2.605,   2.98437),
    'VQF':      (31.458,  2.57,    2.7286),
}

fig, ax = plt.subplots(figsize=(5, 4))

colors = ['#e6194b', '#3cb44b', '#4363d8']
markers = ['o', 's', '^']

for (label, (t, rmse_w, rmse_a)), color, marker in zip(data.items(), colors, markers):
    ax.scatter(t, rmse_w, color=color, marker=marker, s=80, zorder=5, label=f'{label} (clean)')
    ax.scatter(t, rmse_a, color=color, marker=marker, s=80, zorder=5,
               facecolors='none', linewidths=1.8, label=f'{label} (all)')
    # vertical line connecting clean and all for same algorithm
    ax.plot([t, t], [rmse_w, rmse_a], color=color, lw=1, ls='--', alpha=0.5)
    # label next to point
    ax.annotate(label, xy=(t, rmse_w), xytext=(0, -14), textcoords='offset points',
                fontsize=9, color=color, ha='center')

ax.set_xlabel('Computation time (ns/sample)')
ax.set_ylabel('RMSE (°)')
ax.set_title('Algorithm speed vs. accuracy trade-off')

# custom legend: filled = clean noise, open = all noise
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker='o', color='k', markerfacecolor='k', markersize=7, lw=0, label='White list dataset'),
    Line2D([0], [0], marker='o', color='k', markerfacecolor='none', markersize=7,
           markeredgewidth=1.8, lw=0, label='White + disturb datasets (fixed ref)'),
]
ax.legend(handles=legend_elements, fontsize=8)

ax.grid(True, alpha=0.3)
ax.set_xlim(left=0)
ax.set_ylim(bottom=2, top=4.0)

plt.tight_layout()
# plt.savefig(base_fold + 'algo_time_vs_accuracy.pdf', bbox_inches='tight')
plt.savefig(base_fold + 'algo_time_vs_accuracy.png', dpi=300, bbox_inches='tight')
plt.show()
