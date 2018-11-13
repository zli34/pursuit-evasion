"""Analyze the results from an experiment"""

import os

import sys
root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"..")
sys.path.append(os.path.join(root_dir, "build/src/main/"))
import coopPE_pb2

import argparse

def main(data_dir):
    sims = coopPE_pb2.SimStudyData()
    sims_file = os.path.join(data_dir, "sims.pb")
    with open(sims_file, "r") as f:
        sims.ParseFromString(f.read())


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", required = True, type = str)

    args = ap.parse_args()

    main(args.data_dir)
