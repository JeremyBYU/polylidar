import matplotlib.pyplot as plt
from tests.python.helpers.utils import load_csv, load_npy


def plot_points(points):

    fig, ax = plt.subplots(nrows=1, ncols=1)
    ax.scatter(points[:,0], points[:, 1], s=1)
    plt.axis('off')
    ax.axis('equal')

    plt.show()


def main():
    points_1 = load_csv('caholes_8000.csv', delimeter=' ')
    plot_points(points_1)

    points_1 = load_csv('ca_8000.csv', delimeter=' ')
    plot_points(points_1)


if __name__ == "__main__":
    main()