import matplotlib.pyplot as plt
import pandas as pd


if __name__ == '__main__':

    df = pd.read_csv("./saved_datasets/_c_test_rectorat1/gps_positions.csv")

    poli_map = plt.imread("./saved_datasets/_c_test_rectorat1/map.jpg")

    BBox = (df.LON.min(), df.LON.max(),
            df.LAT.min(), df.LAT.max())

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.scatter(df.LON, df.LAT, zorder=1, alpha=0.2, c='blue', s=10)
    ax.set_title('GPS Plot')
    ax.set_xlim(BBox[0], BBox[1])
    ax.set_ylim(BBox[2], BBox[3])
    ax.imshow(poli_map, zorder=0, extent=BBox, aspect='equal')
    plt.axis('off')

    plt.savefig("./saved_datasets/_c_test_rectorat1/coords.jpg")

    plt.show()

    df.plot(kind='line', x='LON', y='LAT', color='red')
    plt.show()
