// MIT License
//
// # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "VDBVolume.h"

// OpenVDB
#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

namespace {

float ComputeSDF(const Eigen::Vector3d& origin,
                 const Eigen::Vector3d& point,
                 const Eigen::Vector3d& voxel_center) {
    const Eigen::Vector3d v_voxel_origin = voxel_center - origin;
    const Eigen::Vector3d v_point_voxel = point - voxel_center;
    const double dist = v_point_voxel.norm();
    const double proj = v_voxel_origin.dot(v_point_voxel);
    const double sign = proj / std::abs(proj);
    return static_cast<float>(sign * dist);
}

Eigen::Vector3d GetVoxelCenter(const openvdb::Coord& voxel, const openvdb::math::Transform& xform) {
    const float voxel_size = xform.voxelSize()[0];
    openvdb::math::Vec3d v_wf = xform.indexToWorld(voxel) + voxel_size / 2.0;
    return Eigen::Vector3d(v_wf.x(), v_wf.y(), v_wf.z());
}

}  // namespace

namespace vdbfusion {

VDBVolume::VDBVolume(float voxel_size, float sdf_trunc, bool space_carving /* = false*/)
    : voxel_size_(voxel_size), sdf_trunc_(sdf_trunc), space_carving_(space_carving) {
    float background = sdf_trunc_;
    if (space_carving == true) background *= -1.0;
    tsdf_ = openvdb::FloatGrid::create(background);
    tsdf_->setName("D(x): signed distance grid");
    tsdf_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    tsdf_->setGridClass(openvdb::GRID_LEVEL_SET);

    tsdf_weights_ = openvdb::FloatGrid::create(0.0f);
    tsdf_weights_->setName("TW(x): tsdf weights grid");
    tsdf_weights_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    tsdf_weights_->setGridClass(openvdb::GRID_UNKNOWN);

    colors_ = openvdb::Vec3SGrid::create(openvdb::Vec3f(0.0f, 0.0f, 0.0f));
    colors_->setName("C(x): colors grid");
    colors_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    colors_->setGridClass(openvdb::GRID_UNKNOWN);

    colors_weights_ = openvdb::FloatGrid::create(0.0f);
    colors_weights_->setName("CW(x): color weights grid");
    colors_weights_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    colors_weights_->setGridClass(openvdb::GRID_UNKNOWN);

    indices_ = openvdb::Int64Grid::create(-1);
    indices_->setName("I(x): indices grid");
    indices_->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    indices_->setGridClass(openvdb::GRID_UNKNOWN);
}

void VDBVolume::UpdateTSDF(const float& sdf,
                           const openvdb::Coord& voxel,
                           const std::function<float(float)>& weighting_function) {
    using AccessorRW = openvdb::tree::ValueAccessorRW<openvdb::FloatTree>;
    if (sdf > -sdf_trunc_) {
        AccessorRW tsdf_acc = AccessorRW(tsdf_->tree());
        AccessorRW tsdf_weights_acc = AccessorRW(tsdf_weights_->tree());
        const float tsdf = std::min(sdf_trunc_, sdf);
        const float weight = weighting_function(sdf);
        tsdf_acc.setValue(voxel, tsdf_acc.getValue(voxel) + tsdf * weight);
        tsdf_weights_acc.setValue(voxel, tsdf_weights_acc.getValue(voxel) + weight);
    }
}

void VDBVolume::Integrate(openvdb::FloatGrid::Ptr grid,
                          const std::function<float(float)>& weighting_function) {
    for (auto iter = grid->cbeginValueOn(); iter.test(); ++iter) {
        const auto& sdf = iter.getValue();
        const auto& voxel = iter.getCoord();
        this->UpdateTSDF(sdf, voxel, weighting_function);
    }
}

void VDBVolume::Integrate(const std::vector<Eigen::Vector3d>& points,
                          const std::vector<Eigen::Vector3d>& colors,
                          const std::vector<uint8_t>& labels,
                          const Eigen::Vector3d& origin,
                          const std::function<float(float)>& weighting_function) {
    if (points.empty()) {
        std::cerr << "PointCloud provided is empty\n";
        return;
    }
    bool has_colors = !colors.empty();
    bool has_labels = !labels.empty();
    if (has_colors && points.size() != colors.size()) {
        std::cerr << "PointCloud and ColorCloud provided do not have the same size\n";
        return;
    }

    // Get some variables that are common to all rays
    const openvdb::math::Transform& xform = tsdf_->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());

    // Get the "unsafe" version of the grid accessors
    auto tsdf_acc = tsdf_->getUnsafeAccessor();
    auto tsdf_weights_acc = tsdf_weights_->getUnsafeAccessor();
    auto colors_acc = colors_->getUnsafeAccessor();
    auto colors_weights_acc = colors_weights_->getUnsafeAccessor();
    auto indices_acc = indices_->getUnsafeAccessor();

    // Iterate points
    for (size_t i = 0; i < points.size(); ++i) {
        // Get the direction from the sensor origin to the point and normalize it
        const auto point = points[i];
        const Eigen::Vector3d direction = point - origin;
        openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
        dir.normalize();

        // Truncate the Ray before and after the source unless space_carving_ is specified.
        const auto depth = static_cast<float>(direction.norm());
        const float t0 = space_carving_ ? 0.0f : depth - sdf_trunc_;
        const float t1 = depth + sdf_trunc_;

        // Create one DDA per ray(per thread), the ray must operate on voxel grid coordinates.
        const auto ray = openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf_);
        openvdb::math::DDA<decltype(ray)> dda(ray);
        do {
            const auto voxel = dda.voxel();
            const auto voxel_center = GetVoxelCenter(voxel, xform);
            const auto sdf = ComputeSDF(origin, point, voxel_center);
            if (sdf > -sdf_trunc_) {
                const float tsdf = std::min(sdf_trunc_, sdf);
                const float weight = weighting_function(sdf);
                tsdf_acc.setValue(voxel, tsdf_acc.getValue(voxel) + tsdf * weight);
                tsdf_weights_acc.setValue(voxel, tsdf_weights_acc.getValue(voxel) + weight);
                if (has_colors) {
                    openvdb::Vec3f color_vec(colors[i][0] * weight, colors[i][1] * weight, colors[i][2] * weight);
                    if (color_vec.isFinite()) {      // no nan or infs
                        colors_acc.setValue(voxel, colors_acc.getValue(voxel) + color_vec);
                        colors_weights_acc.setValue(voxel, colors_weights_acc.getValue(voxel) + weight);
                    }
                }
                if (has_labels) {
                    bool is_active = indices_acc.isValueOn(voxel);
                    if (is_active) labels_store_[indices_acc.getValue(voxel)].push_back(labels[i]);
                    else {
                        indices_acc.setValue(voxel, labels_store_.size());
                        labels_store_.push_back({labels[i]});
                    }
                }
            }
        } while (dda.step());
    }
}

openvdb::FloatGrid::Ptr VDBVolume::Prune(float min_weight) const {
    const auto weights = tsdf_weights_->tree();
    const auto tsdf = tsdf_->tree();
    const auto background = sdf_trunc_;
    openvdb::FloatGrid::Ptr clean_tsdf = openvdb::FloatGrid::create(sdf_trunc_);
    clean_tsdf->setName("D(x): Pruned signed distance grid");
    clean_tsdf->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size_));
    clean_tsdf->setGridClass(openvdb::GRID_LEVEL_SET);
    clean_tsdf->tree().combine2Extended(tsdf, weights, [=](openvdb::CombineArgs<float>& args) {
        if (args.aIsActive() && args.b() > min_weight) {
            args.setResult(args.a());
            args.setResultIsActive(true);
        } else {
            args.setResult(background);
            args.setResultIsActive(false);
        }
    });
    return clean_tsdf;
}
}  // namespace vdbfusion
