//
// Created by jehor on 2025/12/30.
//

#ifndef QTGEOMETRYSIMULATION_MODELLOADER_H
#define QTGEOMETRYSIMULATION_MODELLOADER_H
#include <assimp/Importer.hpp>

#include "units_define.h"
#include "assimp/scene.h"
#include "plane_coordinate_transform.h"

std::vector<float> &operator<<(std::vector<float> &lhs, float rhs);

enum class ModelFormat
{
    WavefrontObj,
    StlBinary,
    DxfBinary,
    PointCloud,
};

struct ModelData
{
    std::vector<Triangle> m_triangleData;

    bool valid{true};
};

class ModelLoader
{
public:
    explicit ModelLoader();

    bool loadModel(const std::string &fileName);

    bool loadModelWithVertexColors(const std::string &fileName);

    [[nodiscard]] const ModelData &getModelData() const;

    [[nodiscard]] const ModelData &getModelData(size_t index) const;

    void popModel();

    void clear();

    [[nodiscard]] size_t size() const;

    void eraseModel(size_t index);

    [[nodiscard]] std::vector<std::string> getArguments() const;


private:
    std::vector<ModelData> m_modelData;

    std::vector<std::string> m_modelPath;

    void processNode(const aiNode *node, const aiScene *scene);

    void processNodeWithVertexColors(const aiNode *node, const aiScene *scene);

    void processMesh(const aiMesh *mesh, const aiScene *scene);

    void processMeshWithVertexColors(const aiMesh *mesh, const aiScene *scene);
};

// std::tuple<size_t, size_t, std::vector<float>> normalizeImage(const std::string &imagePath);
//
// cv::Mat readImage(const std::string &imageFilePath);

/* 插值算法，有绝对插值和相对插值两种插值方式：
 * param0: 需要修改的点阵 vector，要求格式：x, y, z, r, g, b, a 为一个点。
 * param1: 需要进行插值的平面
 * param2: 插值精度（使用绝对插值算法）
 * 精度：
 * 若为绝对精度，则插值使得相邻两个点之间的绝对 OpenGL 归一化距离落在 accuracy 附近，插值数量约为 S(p) / (accuracy ^ 2)
 * param3: 图像数据，要求使用非归一化 [0, 255] RGBA 值。
 * param4: 图像宽度
 * param5: 图像高度
 */
void interpolation_helper(
    std::vector<float> &pointsWithColor,
    const Triangle &p,
    double accuracy,
    const std::vector<float> &rgba,
    size_t cols,
    size_t rows);

// [[nodiscard]] std::vector<float> outputVertexWithColorRGBAArray(const ModelData &modelsData, const char *imagePath, double abs = 0.01);

// 提供一个参考平面（平面点和平面法向量），此函数计算 3D 模型在平面上的投影，并根据投影范围，生成纹理坐标
// 注意，此函数会覆盖原有的纹理坐标信息！
// 因此，只有当 obj 不存在纹理坐标信息时，才能使用此函数
// 提供第三个参数，true 则单面贴图，false 则双面贴图。false 的性能开销更小。
// 提供第四个参数，true 则进行更精细的深度缓冲区处理。此功能的性能开销很大。
// 提供第五个参数，给定贴图范围。
//     本算法计算参考平面旋转到 O-xy 平面后，给定模型的新x、y坐标的最大值和最小值，从而生成四个极值坐标作为纹理坐标生成范围。所有在此范围之内的点将会被生成纹理坐标。
//     第五个参数默认值为 1.0，给定其他的默认值，允许将上述范围进行缩放。
// 提供第六个参数，此参数决定是否“仅贴图一个”。
//     若给定 true，则进行坐标范围判断，仅在范围之内的坐标会生成纹理坐标；
//     若给定 false，则不进行此判断，对所有坐标进行纹理坐标生成，此时将有可能生成多个重复的贴图
void generateTextureCoords(
    const ModelData &modelsData,
    const Eigen::Vector3f &referencePlanePoint,
    const Eigen::Vector3f &referencePlaneNormal,
    bool depthTest = false,
    bool z_bufferEnabled = false,
    double range = 1.0,
    bool onlyOne = true);
#endif // QTGEOMETRYSIMULATION_MODELLOADER_H
