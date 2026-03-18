//
// Created by jehor on 2025/12/30.
//

#ifndef QTGEOMETRYSIMULATION_UNITSDEFINE_H
#define QTGEOMETRYSIMULATION_UNITSDEFINE_H
#include <Eigen/Core>
#include <Eigen/Dense>

#include <optional>

struct VertexInfo
{
    Eigen::Vector3f vertex;
    std::optional<Eigen::Vector2f> texture;
    Eigen::Vector3f normal;
    std::optional<Eigen::Vector4f> color;
};

struct BasicPoint
{
    Eigen::Vector3f vertex;
    Eigen::Vector4f color;
};

enum class PLYFormat
{
    ASCII,
    BinaryLittleEndian,
    BinaryBigEndian
};

enum class DXFFormat
{
    ASCII,
    BinaryLittleEndian,
    BinaryBigEndian
};

using OriginalModel = std::vector<BasicPoint>;

class Triangle
{
public:
    Triangle() = default;
    Triangle(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c);

    [[nodiscard]] Eigen::Vector3f vector0() const;

    [[nodiscard]] Eigen::Vector3f vector1() const;

    [[nodiscard]] double space() const;

    [[nodiscard]] Eigen::Vector3f normal() const;

    void loadNormal();

    VertexInfo vertex_0{};
    VertexInfo vertex_1{};
    VertexInfo vertex_2{};

    // Eigen::Vector3f& vert0 {vertex_0.vertex};
    // Eigen::Vector3f& vert1 {vertex_1.vertex};
    // Eigen::Vector3f& vert2 {vertex_2.vertex};
    //
    // std::optional<Eigen::Vector2f>& tex0{vertex_0.texture};
    // std::optional<Eigen::Vector2f>& tex1{vertex_1.texture};
    // std::optional<Eigen::Vector2f>& tex2{vertex_2.texture};
    //
    // Eigen::Vector3f& normal0{vertex_0.normal};
    // Eigen::Vector3f& normal1{vertex_1.normal};
    // Eigen::Vector3f& normal2{vertex_2.normal};
};
#endif //QTGEOMETRYSIMULATION_UNITSDEFINE_H