#!/usr/bin/env python3
# @file      kitti_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from datasets import ApolloDataset as Dataset
from vdbfusion_pipeline import VDBFusionPipeline as Pipeline


def main(
    root_dir: str,
    sequence: int = 0,
    config: str = "config/apollo.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(root_dir)
    pipeline = Pipeline(dataset, config, jump=jump, n_scans=n_scans, map_name="apollo")
    pipeline.run()
    pipeline.visualize() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
