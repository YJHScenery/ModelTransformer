#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>



/**
 * @brief 坐标系变换类：将平面(P0, n)变换到局部坐标系（P0→原点，n→+z轴）
 * @note GLM实现，列主序矩阵（与OpenGL一致）
 */
class PlaneCoordinateTransform {
public:
    // 构造函数：初始化变换矩阵
    PlaneCoordinateTransform(const Eigen::Vector3f& P0, const Eigen::Vector3f& n) {
        // 1. 法向量归一化
        const Eigen::Vector3f z_local = n.normalized();

        // 2. 构建局部x轴（避免与z_local共线）
        Eigen::Vector3f default_vec(0.0f, 1.0f, 0.0f);
        if (fabs(z_local.dot(default_vec)) > 0.9999f) {
            default_vec = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        }
        const Eigen::Vector3f x_local = default_vec.cross(z_local).normalized();

        // 3. 构建局部y轴（右手坐标系）
        const Eigen::Vector3f y_local = z_local.cross(x_local);

        // 4. 构建旋转矩阵（全局→局部）：列主序
        rotation_matrix_ = Eigen::Matrix4f{};
        // // 第一列：局部X轴
        // rotation_matrix_[0][0] = x_local.x;
        // rotation_matrix_[1][0] = x_local.y;
        // rotation_matrix_[2][0] = x_local.z;
        // // 第二列：局部Y轴
        // rotation_matrix_[0][1] = y_local.x;
        // rotation_matrix_[1][1] = y_local.y;
        // rotation_matrix_[2][1] = y_local.z;
        // // 第三列：局部Z轴
        // rotation_matrix_[0][2] = z_local.x;
        // rotation_matrix_[1][2] = z_local.y;
        // rotation_matrix_[2][2] = z_local.z;
        // 第一列：局部X轴（原有 [行][列] 直接替换为 (行, 列)）
        rotation_matrix_(0, 0) = x_local.x(); // 原 rotation_matrix_[0][0]
        rotation_matrix_(1, 0) = x_local.y(); // 原 rotation_matrix_[1][0]
        rotation_matrix_(2, 0) = x_local.z(); // 原 rotation_matrix_[2][0]

        // 第二列：局部Y轴
        rotation_matrix_(0, 1) = y_local.x(); // 原 rotation_matrix_[0][1]
        rotation_matrix_(1, 1) = y_local.y(); // 原 rotation_matrix_[1][1]
        rotation_matrix_(2, 1) = y_local.z(); // 原 rotation_matrix_[2][1]

        // 第三列：局部Z轴
        rotation_matrix_(0, 2) = z_local.x(); // 原 rotation_matrix_[0][2]
        rotation_matrix_(1, 2) = y_local.y(); // 原 rotation_matrix_[1][2]
        rotation_matrix_(2, 2) = z_local.z(); // 原 rotation_matrix_[2][2]

        // 5. 构建平移矩阵：将P0移至原点
        // translation_matrix_.translate(-P0.x(), -P0.y(), -P0.z());
        translation_matrix_.block<3,1>(0,3) = -P0;

        // 6. 组合变换矩阵：先平移，再旋转
        transform_matrix_ = rotation_matrix_ * translation_matrix_;

        // 7. 构建逆变换矩阵（局部→全局）
        // const Eigen::Matrix4f rotation_inv = rotation_matrix_.transposed(); // 正交矩阵逆=转置
        // Eigen::Matrix4f translation_inv = Eigen::Matrix4f{};
        // translation_inv.translate(P0.x(), P0.y(), P0.z());
        // inv_transform_matrix_ = translation_inv * rotation_inv;
        // rotation_matrix_ 是一个 4x4 正交矩阵（通常为旋转矩阵，无平移部分）
        const Eigen::Matrix4f rotation_inv = rotation_matrix_.transpose();

        // 构造平移矩阵：将点平移 (P0.x, P0.y, P0.z)
        Eigen::Matrix4f translation_inv = Eigen::Matrix4f::Identity();
        translation_inv(0, 3) = P0.x();
        translation_inv(1, 3) = P0.y();
        translation_inv(2, 3) = P0.z();

        // 组合得到逆变换矩阵
        inv_transform_matrix_ = translation_inv * rotation_inv;
    }

    // 变换全局点到局部
    [[nodiscard]] Eigen::Vector3f transformPoint(const Eigen::Vector3f& global_point) const {
        const Eigen::Vector4f homo_point(global_point.x(), global_point.y(), global_point.z(), 1.0f);
        const Eigen::Vector4f homo_local = transform_matrix_ * homo_point;
        return {homo_local.x(), homo_local.y(), homo_local.z()};
    }

    // 变换全局向量到局部（仅旋转）
    [[nodiscard]] Eigen::Vector3f transformVector(const Eigen::Vector3f& global_vec) const {
        const Eigen::Vector4f homo_vec(global_vec.x(), global_vec.y(), global_vec.z(), 0.0f);
        const Eigen::Vector4f homo_local = rotation_matrix_ * homo_vec;
        return {homo_local.x(), homo_local.y(), homo_local.z()};
    }

    // 获取变换矩阵
    [[nodiscard]] Eigen::Matrix4f getTransformMatrix() const { return transform_matrix_; }

    [[nodiscard]] Eigen::Matrix4f getInvTransformMatrix() const { return inv_transform_matrix_; }

private:
    Eigen::Matrix4f rotation_matrix_{};
    Eigen::Matrix4f translation_matrix_{};
    Eigen::Matrix4f transform_matrix_{};
    Eigen::Matrix4f inv_transform_matrix_{};
};
