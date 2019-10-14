import argparse
import itertools
import numpy as np
import os
from multiprocessing import Pool
from tqdm import tqdm
from polylidar import extractPolygonsAndTimings
from polylidarutil import generate_test_points, get_estimated_lmax

def get_combinations(options):
    keys, values = zip(*options.items())
    all_combinations = [dict(zip(keys, v)) for v in itertools.product(*values)]
    return all_combinations

def parse_args():
    parser = argparse.ArgumentParser(description='Explode Polylidar')
    args = parser.parse_args()
    return args


def work(combination):
    kwargs = dict(num_groups=combination['num_groups'], group_size=combination['group_size'], seed=combination['i'])
    points = generate_test_points(**kwargs)
    lmax = get_estimated_lmax(**kwargs)
    polylidar_kwargs = dict(alpha=0.0, lmax=lmax)
    polygons, timings = extractPolygonsAndTimings(points, **polylidar_kwargs)
    time_ms = sum(timings)
    kwargs['time'] = time_ms

def main(num_groups=range(1, 500), group_size=range(100, 1000, 100)):
    options = dict(num_groups=num_groups, group_size=group_size)
    all_combinations = get_combinations(options)
    for i, comb in enumerate(all_combinations):
        comb['i'] = i
    print("Total combinations: {}".format(len(all_combinations)))
    if os.name == 'nt':
        print("Crashes windows. Stick to linux for this test")
        return
    with Pool(8) as p:
        r = list(tqdm(p.imap(work, all_combinations), total=len(all_combinations)))

if __name__ == "__main__":
    args = parse_args()
    main()