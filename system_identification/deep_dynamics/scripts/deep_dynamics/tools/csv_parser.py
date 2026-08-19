#!/usr/bin/env python3
import numpy as np
from tqdm import tqdm
import csv

SAMPLING_TIME = 0.05  # 정보용

def write_dataset(csv_path, horizon, save=True):
    with open(csv_path) as f:
        csv_reader = csv.reader(f, delimiter=",")
        odometry = []  # [vx, vy, omega, throttle, steering]
        poses = []     # [x, y, phi, vx, vy, omega, throttle, steering]
        column_idxs = {}

        for row in csv_reader:
            # 헤더 파싱
            if not column_idxs:
                for i, name in enumerate(row):
                    column_idxs[name.split("(")[0]] = i
                # 수집 노드에서 쓴 헤더와 일치해야 함
                required = [
                    "vx","vy","omega","delta",
                    "throttle_ped_cmd","brake_ped_cmd",
                    "x","y","phi"
                ]
                missing = [k for k in required if k not in column_idxs]
                if missing:
                    raise KeyError(f"CSV missing columns: {missing}")
                continue

            vx = float(row[column_idxs["vx"]])
            # 저속 구간 컷
            if abs(vx) < 0.5:
                continue

            vy = float(row[column_idxs["vy"]])
            omega = float(row[column_idxs["omega"]])
            steering = float(row[column_idxs["delta"]])
            throttle = float(row[column_idxs["throttle_ped_cmd"]])  # duty_cycle 등

            odometry.append([vx, vy, omega, throttle, steering])

            poses.append([
                float(row[column_idxs["x"]]),
                float(row[column_idxs["y"]]),
                float(row[column_idxs["phi"]]),
                vx, vy, omega, throttle, steering
            ])

    odometry = np.asarray(odometry, dtype=np.float64)
    poses = np.asarray(poses, dtype=np.float64)

    if len(odometry) <= horizon:
        raise ValueError(f"sequence({len(odometry)}) <= horizon({horizon})")

    # 순수 슬라이딩 윈도우: feature=(horizon,5), label=다음 시점 (3)
    num_samples = len(odometry) - horizon
    features = np.zeros((num_samples, horizon, 5), dtype=np.float64)
    labels = np.zeros((num_samples, 3), dtype=np.float64)

    for i in tqdm(range(num_samples), desc="Compiling dataset"):
        window = odometry[i:i+horizon]     # (horizon, 5)
        nxt = odometry[i+horizon][:3]      # next [vx, vy, omega]
        features[i] = window
        labels[i] = nxt

    print("Final features shape:", features.shape)  # (N, horizon, 5)
    print("Final labels shape:", labels.shape)      # (N, 3)

    if save:
        out_path = csv_path[:csv_path.find(".csv")] + f"_{horizon}.npz"
        np.savez(out_path, features=features, labels=labels, poses=poses)
    return features, labels, poses


if __name__ == "__main__":
    import argparse, argcomplete
    parser = argparse.ArgumentParser(description="Convert CSV to NPZ (ideal actuator)")
    parser.add_argument("csv_path", type=str)
    parser.add_argument("horizon", type=int)
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    a,b,c = write_dataset(args.csv_path, args.horizon)
    print(a.shape, b.shape, c.shape)
