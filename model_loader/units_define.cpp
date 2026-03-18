//
// Created by jehor on 2025/12/30.
//
#include "units_define.h"

#include <utility>

Triangle::Triangle(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c)
{
    vertex_0.vertex(a), vertex_1.vertex(b), vertex_2.vertex(c);
}

Eigen::Vector3f Triangle::vector0() const
{
    return vertex_1.vertex - vertex_0.vertex;
}

Eigen::Vector3f Triangle::vector1() const
{
    return vertex_2.vertex - vertex_0.vertex;
}

double Triangle::space() const
{
    return 0.5 * vector0().cross(vector1()).norm();
}

Eigen::Vector3f Triangle::normal() const
{
    Eigen::Vector3f vec {vector0().cross(vector1())};
    vec.normalize();
    return vec;
}

void Triangle::loadNormal()
{
    vertex_0.normal = normal();
    vertex_1.normal = vertex_0.normal;
    vertex_2.normal = vertex_1.normal;
}
