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

#include <iostream>
#include <limits>

VoxelPicking::VoxelPicking(
	const std::shared_ptr<open3d::geometry::VoxelGrid> &grid)
{
	mGrid = grid;
	mRenderedGrid = std::make_shared<open3d::geometry::VoxelGrid>(*mGrid);

	CreateVisualizerWindow();
	AddGeometry(mRenderedGrid);
	Run();
}

void VoxelPicking::MouseButtonCallback(GLFWwindow *window, int button,
	int action, int mods)
{
	// Use right button to avoid clashing with camera movements
	const int selectionButton = GLFW_MOUSE_BUTTON_RIGHT;

	if (button == selectionButton && action == GLFW_PRESS) {
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		auto ray = Unproject(xpos, ypos);

		Eigen::Vector3i voxelCoords = GetVoxel(ray);
		bool update = false;

		if (!mods) {
			update = true;
			while (!mSelected.empty()) {
				DeselectVoxel(*(mSelected.begin()));
			}
			update = true;
		}

		if (mods & GLFW_MOD_SHIFT) {
			DeselectVoxel(voxelCoords);
			update = true;
		} else {
			update = SelectVoxel(voxelCoords) || update;
		}

		if (update) {
			AddGeometry(mRenderedGrid, false);
			UpdateRender();
		}
	}

	open3d::visualization::Visualizer::MouseButtonCallback(window, button,
		action, mods);
}

void VoxelPicking::KeyPressCallback(GLFWwindow *window, int key, int scancode,
	int action, int mods)
{
	if (key == GLFW_KEY_DELETE) {
		for (const Eigen::Vector3i coords : mSelected) {
			mGrid->voxels_.erase(coords);
			mRenderedGrid->voxels_.erase(coords);
		}
		mSelected.clear();

		AddGeometry(mRenderedGrid, false);
		UpdateRender();
	}

	open3d::visualization::Visualizer::KeyPressCallback(window, key, scancode,
		action, mods);
}

open3d::geometry::Ray3D VoxelPicking::Unproject(double xpos, double ypos)
{
	using Eigen::Vector3d;
	using namespace open3d::visualization::gl_util;

	// The visualizer lacks a const override for this :(
	auto &vc = GetViewControl();
	GLMatrix4f pvm = vc.GetProjectionMatrix() * vc.GetViewMatrix()
		* vc.GetModelMatrix();
	GLMatrix4f inv = pvm.inverse();
	GLVector4f screenPos(2 * xpos / vc.GetWindowWidth() - 1,
		1 - 2 * ypos / vc.GetWindowHeight(), 1.0f, 1.0f);
	GLVector4f worldPos = inv * screenPos;
	worldPos /= worldPos[3];
	Vector3d eye = vc.GetEye().cast<double>();
	Vector3d dir = Vector3d(worldPos[0], worldPos[1], worldPos[2]) - eye;

	/* We could normalize that direction, but we would not really benefit from
	this, at the moment.
	Also, notice that Open3D for small FOV falls back to orthogonal camera, but
	this code does not take that into account. */
	return {eye, dir};
}

Eigen::Vector3i VoxelPicking::GetVoxel(const open3d::geometry::Ray3D &ray) const
{
	using Eigen::Vector3d;
	using Eigen::Vector3i;
	const Vector3i invalid(-1, -1, -1);

	auto params = IntersectAABB(ray);
	if (params.first >= params.second) {
		return invalid;
	}

	const Vector3d &origin = ray.Origin();
	const Vector3d &direction = ray.Direction();

	/* «A Fast Voxel Traversal Algorithm for Ray Tracing»
	by John Amanatides and Andrew Woo
	http://www.cse.yorku.ca/~amana/research/grid.pdf */

	// The coordinates at which the ray enters the grid, in world units
	Vector3d startWorld = origin + params.first * direction;

	/* Coordinates of the voxel at which the ray enters the grid, in grid units.
	Then used as "current coordinates" */
	Vector3i coords = mGrid->GetVoxel(startWorld);
	// Size of the grid, in grid units
	Vector3i stop = mGrid->GetVoxel(mGrid->GetMaxBound());
	/* Tell if the ray direction is increasing or decreasing along a certain
	direction. This assumes that the voxel grid has the same orientation as the
	world, which is the case in Open3D.
	Notice that -1.0 gets correctly casted to -1. */
	Vector3i step(copysign(1.0, direction[0]), copysign(1.0, direction[1]),
		copysign(1.0, direction[2]));
	// Ray must not be the null vector
	assert(step[0] || step[1] || step[2]);

	/* Size of a voxel, in ray direction paramter coordinates.
	I.e. when starting from a border, what is the line parameter value that
	makes the line go to the next border in that direction? */
	Vector3d tDelta(
		mGrid->voxel_size_ / direction[0] * step[0],
		mGrid->voxel_size_ / direction[1] * step[1],
		mGrid->voxel_size_ / direction[2] * step[2]);

	/* The parameter values at which we change grid row/column/depth.
	This is tMax in the paper but, in my opinion, tNext is a better name for
	this variable. */
	Vector3d tNext;
	for (int i = 0; i < 3; i++) {
		double nextCoord = mGrid->origin_[i] +
			(coords[i] + (step[i] > 0 ? 1 : 0)) * mGrid->voxel_size_;
		tNext[i] = (nextCoord - origin[i]) / direction[i];
	}

	while (mGrid->voxels_.find(coords) == mGrid->voxels_.end()) {
		/* The pseudo code on the paper repeats some similar code a lot, but we
		have indices, so we find the index at which the change happens, then
		the code for the actual action is written once. */
		int dim = 0;
		for (int i = 1; i < 3; i++) {
			if (tNext[i] < tNext[dim]) {
				dim = i;
			}
		}

		coords[dim] += step[dim];
		if (coords[dim] > stop[dim] || coords[dim] < 0) {
			return invalid;
		}
		tNext[dim] += tDelta[dim];
	}

	/* If we arrive here, coords is valid, or we would have not exited the while
	loop. */
	return coords;
}

std::pair<double, double> VoxelPicking::IntersectAABB(
	const open3d::geometry::Ray3D &ray) const
{
	using Eigen::Vector3d;

	const Vector3d &origin = ray.Origin();
	const Vector3d &direction = ray.Direction();

	Vector3d min = mGrid->GetMinBound();
	Vector3d max = mGrid->GetMaxBound();

	double tmin = 0;
	double tmax = std::numeric_limits<double>::max();

	// http://psgraphics.blogspot.com/2016/02/new-simple-ray-box-test-from-andrew.html
	for (int i = 0; i < 3; i++) {
		double invD = 1.0f / direction[i];
		double t0 = (min[i] - origin[i]) * invD;
		double t1 = (max[i] - origin[i]) * invD;
		if (invD < 0) {
			std::swap(t0, t1);
		}

		if (t0 > tmin) {
			tmin = t0;
		}
		if (t1 < tmax) {
			tmax = t1;
		}

		if (tmax <= tmin) {
			break;
		}
	}

	return std::make_pair(tmin, tmax);
}

bool VoxelPicking::SelectVoxel(const Eigen::Vector3i &coords)
{
	auto it = mRenderedGrid->voxels_.find(coords);
	if (it != mRenderedGrid->voxels_.end() && !mSelected.count(coords)) {
		mSelected.insert(coords);
		it->second.color_ = Eigen::Vector3d(1.0, 0, 0);
		return true;
	}

	return false;
}

void VoxelPicking::DeselectVoxel(const Eigen::Vector3i &coords)
{
	auto original = mGrid->voxels_.find(coords);
	auto maybeChanged = mRenderedGrid->voxels_.find(coords);

	/* Erase after searching, as coords may be from the set, and the operation
	invalidates it in some cases.
	As an alternative, take coords as value, instead of reference.*/
	mSelected.erase(coords);

	if (original == mGrid->voxels_.end()
			|| maybeChanged == mRenderedGrid->voxels_.end()) {
		return;
	}

	maybeChanged->second = original->second;
}
