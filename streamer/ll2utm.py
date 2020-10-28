import pandas as pd
import numpy as np

# rad- degrees
RADIANS_PER_DEGREE = np.pi / 180.0
DEGREES_PER_RADIAN = 180.0 / np.pi

#  WGS84 Parameters
WGS84_A = 6378137.0  # major axis
WGS84_B = 6356752.31424518  # minor axis
WGS84_F = 0.0033528107  # ellipsoid flattening
WGS84_E = 0.0818191908  # first eccentricity
WGS84_EP = 0.0820944379  # second eccentricity

# UTM Parameters
UTM_K0 = 0.9996  # scale factor
UTM_FE = 500000.0  # false easting
UTM_FN_N = 0.0  # false northing, northern hemisphere
UTM_FN_S = 10000000.0  # false northing, southern hemisphere
UTM_E2 = (WGS84_E * WGS84_E)  # e^2
UTM_E4 = (UTM_E2 * UTM_E2)  # e^4
UTM_E6 = (UTM_E4 * UTM_E2)  # e^6
UTM_EP2 = (UTM_E2 / (1 - UTM_E2))  # e'^2

# Conversion constants
M0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256)
M1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024)
M2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024)
M3 = -(35 * UTM_E6 / 3072)


def ll2utm(latitude, longitude):
    # Central Meridian
    cm = 0.0
    if longitude >= 0.0:
        cm = int(longitude) - (int(longitude)) % 6 + 3
    else:
        cm = int(longitude) - (int(longitude)) % 6 - 3

    # convert degrees into radians double
    rlat = latitude * RADIANS_PER_DEGREE
    rlon = longitude * RADIANS_PER_DEGREE
    rlon0 = cm * RADIANS_PER_DEGREE

    # compute trigonometric functions
    slat = np.sin(rlat)
    clat = np.cos(rlat)
    tlat = np.tan(rlat)

    # decide the false northing at origin
    fn = UTM_FN_N if latitude > 0.0 else UTM_FN_S

    T = tlat ** 2
    C = UTM_EP2 * clat ** 2
    A = (rlon - rlon0) * clat
    M = WGS84_A * (M0 * rlat + M1 * np.sin(2.0 * rlat) +
                   M2 * np.sin(4.0 * rlat) + M3 * np.sin(6.0 * rlat))
    V = WGS84_A / np.sqrt(1.0 - UTM_E2 * slat * slat)

    # Compute the easting-northing coordinates
    x = UTM_FE + \
        UTM_K0 * V * (A + (1.0 - T + C) * np.power(A, 3) / 6.0 + \
                      (5.0 - 18.0 * T + (T ** 2) + 72.0 * C - 58.0 * UTM_EP2) * np.power(A, 5) / 120.0)

    y = fn + UTM_K0 * \
        (M + V * tlat * ((A ** 2) / 2.0 + (5.0 - T + 9.0 * C + 4.0 * (C ** 2)) * \
        np.power(A, 4) / 24.0 + ((61 - 58 * T + (T ** 2) + 600 * C - 330.0 * UTM_EP2) * np.power(A, 6) / 720.0)))

    return x, y


# Poli must be at 424882.84 mE 4920748.89 mN with cm resolution
# x, y = ll2utm(44.436139, 26.056134)
# print(x, y)

# TODO: Remove this
lat_lon_df = pd.read_csv('./saved_datasets/gps_enclosed_2m_square.csv')
print(lat_lon_df.head())
lat = lat_lon_df['LAT'].to_numpy()
lon = lat_lon_df['LON'].to_numpy()

utm_df = pd.DataFrame([ll2utm(i, j) for i, j in zip(lat, lon)], columns=['x', 'y'])
print(utm_df.head())

print(utm_df[['x', 'y']].agg([np.mean, np.var]))



