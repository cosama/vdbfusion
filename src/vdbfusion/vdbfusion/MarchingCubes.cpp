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

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>

#include "MarchingCubesConst.h"
#include "VDBVolume.h"

namespace openvdb {
static const openvdb::Coord shift[8] = {
    openvdb::Coord(0, 0, 0), openvdb::Coord(1, 0, 0), openvdb::Coord(1, 1, 0),
    openvdb::Coord(0, 1, 0), openvdb::Coord(0, 0, 1), openvdb::Coord(1, 0, 1),
    openvdb::Coord(1, 1, 1), openvdb::Coord(0, 1, 1),
};
}

// Taken from <open3d/utility/Eigen.h>
namespace {
template <typename T>
struct hash_eigen {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (int i = 0; i < (int)matrix.size(); i++) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
}  // namespace

namespace vdbfusion {

static constexpr int const X = 0;
static constexpr int const Y = 1;
static constexpr int const Z = 2;
static constexpr int const SOURCE = 0;
static constexpr int const DEST = 1;

template <typename T>
using Accessor = typename openvdb::tree::ValueAccessor<T, true>;

uint8_t MostCommon(std::vector<uint8_t> store) {
    std::unordered_map<uint8_t, int> counts;
    for (auto it = store.begin(); it != store.end(); ++it) {
        if (counts.find(*it) != counts.end()) {
            ++counts[*it];
        }
        else {
            counts[*it] = 1;
        }
    }
    return std::max_element(counts.begin(), counts.end(),
            [] (const std::pair<uint8_t, int>& pair1, const std::pair<uint8_t, int>& pair2) {
            return pair1.second < pair2.second;})->first;
}

template <typename T, typename Y, typename C, typename I>
int GetCubeIndex(const openvdb::Coord& voxel,
                 bool fill_holes,
                 float min_weight,
                 Accessor<T>& tsdf_weights_acc,
                 Accessor<Y>& tsdf_acc,
                 Accessor<T>& colors_weights_acc,
                 Accessor<C>& colors_acc,
                 Accessor<I>& indices_acc,
                 float* vertex_tsdf,
                 openvdb::Vec4f* vertex_colors,
                 int* indices) {
    int cube_index = 0;
    // Iterate through all the 8 neighbour vertices...
    for (int vertex = 0; vertex < 8; vertex++) {
        openvdb::Coord idx = voxel + openvdb::shift[vertex];
        float weight = tsdf_weights_acc.getValue(idx);
        float tsdf = tsdf_acc.getValue(idx);
        if (!fill_holes) {
            if (weight == 0.0f) {
                cube_index = 0;
                break;
            }
        }
        if (weight < min_weight) {
            cube_index = 0;
            break;
        }
        if (tsdf < 0.0f) {
            cube_index |= (1 << vertex);
        }
        vertex_tsdf[vertex] = tsdf / weight;
        auto c = colors_acc.getValue(idx);
        vertex_colors[vertex] = openvdb::Vec4f(c[0], c[1], c[2], colors_weights_acc.getValue(idx));
        indices[vertex] = indices_acc.getValue(idx);
    }
    return cube_index;
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>, std::vector<Eigen::Vector3d>, std::vector<uint8_t>>
VDBVolume::ExtractTriangleMesh(bool fill_holes, float min_weight) const {
    // implementation of marching cubes, based on Open3D
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    std::vector<Eigen::Vector3d> colors;
    std::vector<uint8_t> labels;

    double half_voxel_length = voxel_size_ * 0.5;
    // Map of "edge_index = (x, y, z, 0) + edge_shift" to "global vertex index"
    std::unordered_map<Eigen::Vector4i, int, hash_eigen<Eigen::Vector4i>, std::equal_to<>,
                       Eigen::aligned_allocator<std::pair<const Eigen::Vector4i, int>>>
        edgeindex_to_vertexindex;
    int edge_to_index[12];

    auto colors_acc = colors_->getAccessor();
    auto tsdf_acc = tsdf_->getAccessor();
    auto tsdf_weights_acc = tsdf_weights_->getAccessor();
    auto colors_weights_acc = colors_weights_->getAccessor();
    auto indices_acc = indices_->getAccessor();
    for (auto iter = tsdf_->beginValueOn(); iter; ++iter) {
        float vertex_tsdf[8];
        openvdb::Vec4f colors_field[8];
        int indices_field[8];
        const openvdb::Coord& voxel = iter.getCoord();
        const int32_t x = voxel.x();
        const int32_t y = voxel.y();
        const int32_t z = voxel.z();
        int cube_index = GetCubeIndex(voxel, fill_holes, min_weight, tsdf_weights_acc, tsdf_acc,
                                      colors_weights_acc, colors_acc, indices_acc,
                                      vertex_tsdf, colors_field, indices_field);
        if (cube_index == 0 || cube_index == 255) {
            continue;
        }
        // Iterate trough all the edges..
        for (int edge = 0; edge < 12; edge++) {
            if ((edge_table[cube_index] & (1 << edge)) != 0) {
                Eigen::Vector4i edge_index = Eigen::Vector4i(x, y, z, 0) + edge_shift[edge];
                if (edgeindex_to_vertexindex.find(edge_index) == edgeindex_to_vertexindex.end()) {
                    edge_to_index[edge] = (int)vertices.size();
                    edgeindex_to_vertexindex[edge_index] = (int)vertices.size();
                    // set point to source vertex (x, y, z) coordinates
                    Eigen::Vector3d point(half_voxel_length + voxel_size_ * edge_index(X),
                                          half_voxel_length + voxel_size_ * edge_index(Y),
                                          half_voxel_length + voxel_size_ * edge_index(Z));
                    // source vertex TSDF
                    double source_tsdf = std::abs((double)vertex_tsdf[edge_to_vert[edge][SOURCE]]);
                    // destination vertex TSDF
                    double destination_tsdf =
                        std::abs((double)vertex_tsdf[edge_to_vert[edge][DEST]]);
                    // adding delta to reach destination vertex
                    point(edge_index(3)) +=
                        source_tsdf * voxel_size_ / (source_tsdf + destination_tsdf);
                    vertices.push_back(point /* + origin_*/);
                    // colors
                    const auto& source_color = colors_field[edge_to_vert[edge][SOURCE]];
                    const auto& destination_color = colors_field[edge_to_vert[edge][DEST]];
                    openvdb::Vec3f mix_color = (source_color.getVec3() + destination_color.getVec3()) / (source_color[3] + destination_color[3]);
                    colors.push_back({mix_color[0], mix_color[1], mix_color[2]});
                    // labels
                    std::vector<uint8_t> all_labels;
                    int ind = -1;
                    ind = indices_field[edge_to_vert[edge][SOURCE]];
                    if (ind > -1) {
                        auto this_labels = labels_store_[indices_field[edge_to_vert[edge][SOURCE]]];
                        all_labels.insert(all_labels.end(), this_labels.begin(), this_labels.end());
                    }
                    ind = indices_field[edge_to_vert[edge][DEST]];
                    if (ind > -1) {
                        auto this_labels = labels_store_[indices_field[edge_to_vert[edge][DEST]]];
                        all_labels.insert(all_labels.end(), this_labels.begin(), this_labels.end());
                    }
                    if (all_labels.size() > 0) labels.push_back(MostCommon(all_labels));
                } else {
                    edge_to_index[edge] = edgeindex_to_vertexindex.find(edge_index)->second;
                }
            }
        }
        for (int i = 0; tri_table[cube_index][i] != -1; i += 3) {
            triangles.emplace_back(edge_to_index[tri_table[cube_index][i]],
                                   edge_to_index[tri_table[cube_index][i + 2]],
                                   edge_to_index[tri_table[cube_index][i + 1]]);
        }
    }
    return std::make_tuple(vertices, triangles, colors, labels);
}

}  // namespace vdbfusion
