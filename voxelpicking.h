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

#pragma once

#include <open3d/Open3D.h>

class VoxelPicking : public open3d::visualization::Visualizer {
public:
	/**
	 * Setup a visualizer for a voxel grid
	 *
	 * \param grid The grid to show/modify. Please notice that althoguh the
	 * reference is constant, the pointed type is not, and might be modified
	 */
	VoxelPicking(const std::shared_ptr<open3d::geometry::VoxelGrid> &grid);

protected:
	void MouseButtonCallback(GLFWwindow *window, int button, int action,
		int mods) override;

	void KeyPressCallback(GLFWwindow *window, int key, int scancode, int action,
		int mods) override;

private:

	/**
	 * Unproject a point on the view.
	 *
	 * \param xpos, ypos Position in window coordinates
	 * \return A ray that goes from the camera to the passed point
	 */
	open3d::geometry::Ray3D Unproject(double xpos, double ypos);

	/**
	 * Get the coordinates of the first voxel that the ray intersects.
	 *
	 * \return The ray coordinates, or (-1, -1, -1) if no voxel has been found
	 */
	Eigen::Vector3i GetVoxel(const open3d::geometry::Ray3D &ray) const;

	/**
	 * Intersect a ray with the axis-aligned bounding box of the voxel grid.
	 *
	 * \param ray The ray to intersect
	 * \return The min and max parameter for the line. If second is >= first,
	 * the ray does not intersect the volume.
	 */
	std::pair<double, double> IntersectAABB(
		const open3d::geometry::Ray3D &ray) const;

	/**
	 * Select a voxel.
	 *
	 * \param coords The voxel coordinates, in grid units
	 * \return true if the render needs to be updated
	 */
	bool SelectVoxel(const Eigen::Vector3i &coords);

	/**
	 * Deselect a voxel.
	 *
	 * \param coords The voxel coordinates, in grid units
	 */
	void DeselectVoxel(const Eigen::Vector3i &coords);

	/// The original voxel grid.
	std::shared_ptr<open3d::geometry::VoxelGrid> mGrid;

	/// A copy of the grid, that we can modify (e.g. change color for selection)
	std::shared_ptr<open3d::geometry::VoxelGrid> mRenderedGrid;

	/// The coordinates of selected voxels
	std::unordered_set<Eigen::Vector3i,
		open3d::utility::hash_eigen<Eigen::Vector3i>> mSelected;
};
