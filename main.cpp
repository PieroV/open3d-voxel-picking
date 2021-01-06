/**
 * Visualizer with voxel picking for Open3D.
 *
 * Written in 2021 by Pier Angelo Vendrame <vogliadifarniente AT gmail DOT com>
 *
 * To the extent possible under law, the author has dedicated all copyright
 * and related and neighboring rights to this software to the public domain
 * worldwide. This software is distributed without any warranty.
 *
 * If your country does not recognize the public domain, or if you need a
 * license, please refer to Creative Commons CC0 Public Domain Dedication
 * <http://creativecommons.org/publicdomain/zero/1.0>.
 */

#include "voxelpicking.h"

#include <cstdio>

int main(int argc, char *argv[])
{
	open3d::geometry::PointCloud pcd;
	double size = argc >= 3 ? strtod(argv[2], nullptr) : 0;

	if (argc < 3 || !open3d::io::ReadPointCloud(argv[1], pcd) || size <= 0) {
		fprintf(stderr, "Usage: %s point-cloud-file voxel-size\n", argv[0]);
		return 1;
	}

	auto grid = open3d::geometry::VoxelGrid::CreateFromPointCloud(pcd, size);
	VoxelPicking visualizer(grid);

	return 0;
}
