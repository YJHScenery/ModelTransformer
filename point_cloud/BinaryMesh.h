//
// Created by jehor on 2026/3/18.
//

#ifndef MODELTRANSFORMER_BINARYMESH_H
#define MODELTRANSFORMER_BINARYMESH_H
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <ranges>
#include <unsupported/Eigen/CXX11/Tensor>
#include "units_define.h"
#include <random>

struct BasicMesh
{
    int &get(size_t x, size_t y, size_t z);

    std::array<int, 64> meshData{};
};

struct TwoDMesh
{
    int &get(size_t x, size_t y);

    std::array<int, 16> meshData{};
};

class BinaryMesh
{
public:
    /**
     *
     * @param data 输入的完成基础三维点云数据
     * @param resolution 分辨率，决定网格大小。其值表达采样点生成网格的密度
     */
    BinaryMesh(const OriginalModel &data, double resolution);

    /**
     *
     */
    void process();

    bool outputDXF(const std::string &fileName, DXFFormat format = DXFFormat::ASCII);

private:
    //! @brief 三维张量表达网格信息，处理之后每个位置表达一个灰度。最后将根据此处理
    Eigen::Tensor<int, 3> m_originalMesh;

    //! @brief 二值化后的数据，三维张量通过坐标表达位置，
    //! @details 每个位置存储的是此处通过二值化算法生成的 4x4x4 或 4x4x1 的三维网格。暂且通过 BasicMesh 类表达信息。
    Eigen::Tensor<BasicMesh, 3> m_processingMesh{};

    static int toGray(const Eigen::Vector4f &normalizeColor);
};

class TowDBinaryMesh
{
public:
    /**
     *
     * @param data 输入的完成基础三维点云数据
     * @param resolution 分辨率，决定网格大小。其值表达采样点生成网格的密度
     */
    TowDBinaryMesh(const OriginalModel &data, double resolution);

    /**
     *
     */
    void process();

    bool outputDXF(const std::string &fileName, DXFFormat format = DXFFormat::ASCII);

private:
    //! @brief 三维张量表达网格信息，处理之后每个位置表达一个灰度。最后将根据此处理
    Eigen::Tensor<int, 3> m_originalMesh;

    //! @brief 二值化后的数据，三维张量通过坐标表达位置，
    //! @details 每个位置存储的是此处通过二值化算法生成的 4x4x1 的二维网格。通过 TwoDMesh 类表达信息。
    Eigen::Tensor<TwoDMesh, 3> m_processingMesh{};

    static int toGray(const Eigen::Vector4f &normalizeColor);
};

#endif // MODELTRANSFORMER_BINARYMESH_H