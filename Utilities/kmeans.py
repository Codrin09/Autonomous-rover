## Initialisation

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import copy

def assignment(df, centroids, colmap):
    for i in centroids.keys():
        # sqrt((x1 - x2)^2 - (y1 - y2)^2)
        df['distance_from_{}'.format(i)] = (
            np.sqrt(
                (df['x'] - centroids[i][0]) ** 2
                + (df['y'] - centroids[i][1]) ** 2
            )
        )
    centroid_distance_cols = ['distance_from_{}'.format(i) for i in centroids.keys()]
    df['closest'] = df.loc[:, centroid_distance_cols].idxmin(axis=1)
    df['closest'] = df['closest'].map(lambda x: int(x.lstrip('distance_from_')))
    df['color'] = df['closest'].map(lambda x: colmap[x])
    return df

def update(df, centroids):
    for i in centroids.keys():
        centroids[i][0] = np.mean(df[df['closest'] == i]['x'])
        centroids[i][1] = np.mean(df[df['closest'] == i]['y'])
    return centroids

def k_means(df, centroids, colmap):
    # Continue until all assigned categories don't change any more
    while True:
        closest_centroids = df['closest'].copy(deep=True)
        centroids = update(df, centroids)
        df = assignment(df, centroids, colmap)
        if closest_centroids.equals(df['closest']):
            return df, centroids

if __name__ == "__main__":
    x = []
    y = []

    np.random.seed(200)

    for i in range(1000):
        x.append(np.random.randint(0,1000))
        y.append(np.random.randint(0,1000))

    df = pd.DataFrame()
    df['x'] = y
    df['y'] = x

    k = 4
    # centroids[i] = [x, y]
    centroids = {
        i+1: [np.random.randint(0, 1000), np.random.randint(0, 1000)]
        for i in range(k)
    }
    colmap = {1: 'r', 2: 'g', 3: 'b', 4:'k'}

    ## Assignment Stage
    df = assignment(df, centroids, colmap)
    print(df.head())

    df, centroids = k_means(df, centroids, colmap)

    fig = plt.figure(figsize=(5, 5))
    plt.scatter(df['x'], df['y'], color=df['color'], alpha=0.5, edgecolor='k')
    for i in centroids.keys():
        plt.scatter(*centroids[i], color=colmap[i])
    plt.xlim(0, 1000)
    plt.ylim(0, 1000)
    plt.show()