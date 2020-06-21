import json 
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

sns.set(font_scale=1.15)
sns.set_style('white')
sns.set_style("ticks")



def plot_df(df, name='basement'):
    fig = plt.figure(figsize=(6,3.5))
    ax = sns.lineplot(x="threads", y="Speedup", hue="normals", palette='deep', data=df, legend=False)
    ax2 = ax.twinx()
    sns.lineplot(x="threads", y="Execution Time (ms)", ax=ax2, hue='normals',palette='muted', data=df, legend=False)
    for line in ax2.lines:
        line.set_linestyle('--')

    current_palette = sns.color_palette()
    from matplotlib.lines import Line2D
    custom_lines = [Line2D([0], [0], color=current_palette[i], lw=1) for i in range(4)]
    custom_lines_2 = [Line2D([0], [0], color='k', lw=1),
                    Line2D([0], [0], color='k', lw=1, dashes=[2,6])]

    ax.legend(custom_lines, ['1 DP', '2 DP', '3 DP', '4 DP'], loc='upper left', bbox_to_anchor=(0.1, 1,))
    ax2.legend(custom_lines_2, ['Speedup', 'Execution Time'], loc='upper left', bbox_to_anchor=(0.35, 1,))
    # ax.axis('equal')
    # ax.legend(custom_lines, ['1 DP', '2 DP', '3 DP', '4 DP'], loc='upper left', bbox_to_anchor=(0.4, 0.4,))
    # ax2.legend(custom_lines_2, ['Speedup', 'Execution Time'], loc='upper left', bbox_to_anchor=(0.6, 0.4,))
    fig.savefig(f'assets/scratch/{name}_speedup.pdf', bbox_inches='tight')
    plt.tight_layout()
    plt.show()


def main():
    with open('bench/records/bench_mesh_paper.json') as json_file:
        data = json.load(json_file)
    benchmarks = data['benchmarks']

    benchmarks_filtered = []

    for i, bench in enumerate(benchmarks):
        name = bench['name']
        run_name = bench['run_name']
        aggregate_name = bench['aggregate_name']
        if i % 3 == 0:
            names = name.split('/')
            bench['mesh'] = names[0]
            bench['normals'] = int(names[2])
            bench['threads'] = int(names[3])
            bench['time_mean'] = bench['real_time'] if bench['time_unit'] == 'ms' else bench['real_time'] / 1000
            bench_median = benchmarks[i+1]
            bench['time_median'] = bench_median['real_time'] if bench_median['time_unit'] == 'ms' else bench_median['real_time'] / 1000
            bench_std = benchmarks[i+2]
            bench['time_std'] = bench_std['real_time'] if bench_std['time_unit'] == 'ms' else bench_std['real_time'] / 1000
            benchmarks_filtered.append(bench)
    # calculate speedup
    current_speed = None
    for i, bench in enumerate(benchmarks_filtered):
        # reset current speed every 8 iterations (8 threads)
        if i % 8 == 0:
            current_speed = bench['time_mean']
        bench['speedup'] = current_speed / bench['time_mean']

    df = pd.DataFrame.from_records(benchmarks_filtered)
    df_reduced = df[['mesh', 'normals', 'threads', 'speedup', 'time_mean', 'time_median', 'time_std']]
    df_reduced = df_reduced.rename(columns={'time_mean': "Execution Time (ms)", "speedup": "Speedup"})

    df_sparse = df_reduced[df_reduced['mesh'] == 'SparseMeshPaper']
    df_dense = df_reduced[df_reduced['mesh'] == 'DenseMeshPaper']

    plot_df(df_sparse, 'basement')
    plot_df(df_dense, 'mainfloor')

if __name__ == "__main__":
    main()