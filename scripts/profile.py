import argparse
import os
import numpy as np
from polylidar import extractPolygonsAndTimings
from polylidarutil import generate_test_points, get_estimated_lmax

THIS_DIR = os.path.dirname(__file__)
FIXTURES_DIR = os.path.join(THIS_DIR, '..', 'tests', 'fixtures')

def gen_file_name(num_groups, group_size, lmax):
    return "ng_{}_gs_{}_lmax_{:.3f}.csv".format(num_groups, group_size, lmax)

def parse_args():
    parser = argparse.ArgumentParser(description='Profile Polylidar')
    parser.add_argument('--num_groups', help='Number of groups', default=1, type=int)
    parser.add_argument('--group_size', help='Number of groups', default=100_000, type=int)
    parser.add_argument('--save', help='Save data', action='store_true')
    args = parser.parse_args()
    return args

def main(num_groups=1, group_size=100_000, save=False):
    kwargs = dict(num_groups=num_groups, group_size=group_size, seed=1)
    # generate random normally distributed clusters of points, 200 X 2 numpy array.
    points = generate_test_points(**kwargs)
    lmax = get_estimated_lmax(**kwargs)
    polylidar_kwargs = dict(alpha=0.0, lmax=lmax)
    polygons, timings = extractPolygonsAndTimings(points, **polylidar_kwargs)
    print(timings)
    if save:
        fpath = os.path.join(FIXTURES_DIR, gen_file_name(num_groups, group_size, lmax))
        np.savetxt(fpath, points, fmt='%.4f', header="X,Y") 

if __name__ == "__main__":
    args = parse_args()
    main(num_groups=args.num_groups, group_size=args.group_size, save=args.save)