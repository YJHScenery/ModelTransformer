//
// Created by jehor on 2025/12/30.
//
#include "model_loader.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <iostream>

ModelLoader::ModelLoader() = default;

bool ModelLoader::loadModel(const std::string &fileName)
{

    Assimp::Importer importer;

    constexpr unsigned int postProcessFlags{aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_FlipUVs};

    const aiScene *scene{importer.ReadFile(fileName.c_str(), postProcessFlags)};
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cout << "Cannot load Model File:" << fileName;
        return false;
    }

    processNode(scene->mRootNode, scene);
    m_modelPath.push_back(fileName);
    return true;
}

bool ModelLoader::loadModelWithVertexColors(const std::string &fileName)
{
    Assimp::Importer importer;

    constexpr unsigned int postProcessFlags{aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_FlipUVs};

    const aiScene *scene{importer.ReadFile(fileName.c_str(), postProcessFlags)};
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        std::cout << "Cannot load Model File:" << fileName;
        return false;
    }

    processNodeWithVertexColors(scene->mRootNode, scene);
    m_modelPath.push_back(fileName);
    return true;
}

const ModelData &ModelLoader::getModelData() const
{
    static ModelData emptyModelData{};
    if (!m_modelData.empty())
    {
        return m_modelData.back();
    }
    std::cerr << "Model Data is empty: " << __func__;
    return emptyModelData;
}

const ModelData &ModelLoader::getModelData(const size_t index) const
{
    static ModelData emptyModelData{};
    if (!m_modelData.empty())
    {
        return m_modelData[index];
    }
    std::cerr << "Model Data is empty: " << __func__;
    return emptyModelData;
}

void ModelLoader::popModel()
{
    m_modelData.pop_back();
    m_modelPath.pop_back();
}

void ModelLoader::clear()
{
    m_modelData.clear();
    m_modelPath.clear();
}

size_t ModelLoader::size() const
{
    return m_modelData.size();
}

void ModelLoader::eraseModel(const size_t index)
{
    if (index < m_modelData.size())
    {
        m_modelData.erase(m_modelData.begin() + static_cast<long long>(index));
        m_modelPath.erase(m_modelPath.begin() + static_cast<long long>(index));
    }
}

std::vector<std::string> ModelLoader::getArguments() const
{
    return m_modelPath;
}


void ModelLoader::processNode(const aiNode *node, const aiScene *scene)
{
    for (unsigned int i{0}; i < node->mNumMeshes; ++i)
    {
        const aiMesh *mesh{scene->mMeshes[node->mMeshes[i]]};
        processMesh(mesh, scene);
    }
    for (unsigned int i{0}; i < node->mNumChildren; ++i)
    {
        processNode(node->mChildren[i], scene);
    }
}

void ModelLoader::processNodeWithVertexColors(const aiNode *node, const aiScene *scene)
{
    for (unsigned int i{0}; i < node->mNumMeshes; ++i)
    {
        const aiMesh *mesh{scene->mMeshes[node->mMeshes[i]]};
        processMeshWithVertexColors(mesh, scene);
    }
    for (unsigned int i{0}; i < node->mNumChildren; ++i)
    {
        processNodeWithVertexColors(node->mChildren[i], scene);
    }
}

void ModelLoader::processMesh(const aiMesh *mesh, const aiScene *scene)
{
    ModelData model;

    // 遍历网格的所有面（已被Assimp三角化，每个面固定3个索引）
    for (unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        const aiFace &face{mesh->mFaces[i]};
        Triangle triangle;

        // 1. 填充顶点数据（转换aiVector3D到QVector3D）
        for (unsigned int j = 0; j < 3; j++)
        {
            const aiVector3D &aiVert{mesh->mVertices[face.mIndices[j]]};
            const Eigen::Vector3f qVert(aiVert.x, aiVert.y, aiVert.z);
            switch (j)
            {
            case 0:
                triangle.vertex_0.vertex = qVert;
                break;
            case 1:
                triangle.vertex_1.vertex = qVert;
                break;
            case 2:
                triangle.vertex_2.vertex = qVert;
                break;
            default:;
            }
        }

        // 2. 填充纹理坐标（若网格无纹理坐标，保持std::optional为空）
        if (mesh->mTextureCoords[0] != nullptr)
        {
            // Assimp支持多套纹理坐标，取第一套
            for (unsigned int j = 0; j < 3; j++)
            {
                const aiVector3D &aiTex{mesh->mTextureCoords[0][face.mIndices[j]]};
                Eigen::Vector2f qTex(aiTex.x, aiTex.y);
                switch (j)
                {
                case 0:
                    triangle.vertex_0.texture = qTex;
                    break;
                case 1:
                    triangle.vertex_1.texture = qTex;
                    break;
                case 2:
                    triangle.vertex_2.texture = qTex;
                    break;
                default:;
                }
            }
        }
        else
        {
            // 无纹理坐标，std::optional默认为空，无需额外操作
            triangle.vertex_0.texture.reset();
            triangle.vertex_1.texture.reset();
            triangle.vertex_2.texture.reset();
        }

        // 3. 填充法向量（若网格无法线，调用loadNormal自动生成；否则直接转换）
        if (mesh->mNormals != nullptr)
        {
            // 模型自带法向量，直接转换填充
            for (unsigned int j = 0; j < 3; j++)
            {
                const aiVector3D &aiNormal{mesh->mNormals[face.mIndices[j]]};
                const Eigen::Vector3f qNormal(aiNormal.x, aiNormal.y, aiNormal.z);
                switch (j)
                {
                case 0:
                    triangle.vertex_0.normal = qNormal;
                    break;
                case 1:
                    triangle.vertex_1.normal = qNormal;
                    break;
                case 2:
                    triangle.vertex_2.normal = qNormal;
                    break;
                default:;
                }
            }
        }
        else
        {
            // 模型无法线，调用loadNormal生成（确保不留空）
            triangle.loadNormal();
        }

        // 4. 将当前三角形加入列表
        model.m_triangleData.push_back(triangle);
    }
    m_modelData.push_back(model);
}

void ModelLoader::processMeshWithVertexColors(const aiMesh *mesh, const aiScene *scene)
{
    ModelData model;

    for (unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        const aiFace &face{mesh->mFaces[i]};
        Triangle triangle;

        for (unsigned int j = 0; j < 3; j++)
        {
            const aiVector3D &aiVert{mesh->mVertices[face.mIndices[j]]};
            const Eigen::Vector3f qVert(aiVert.x, aiVert.y, aiVert.z);
            switch (j)
            {
            case 0:
                triangle.vertex_0.vertex = qVert;
                break;
            case 1:
                triangle.vertex_1.vertex = qVert;
                break;
            case 2:
                triangle.vertex_2.vertex = qVert;
                break;
            default:;
            }
        }

        triangle.vertex_0.texture.reset();
        triangle.vertex_1.texture.reset();
        triangle.vertex_2.texture.reset();

        if (mesh->mNormals != nullptr)
        {
            for (unsigned int j = 0; j < 3; j++)
            {
                const aiVector3D &aiNormal{mesh->mNormals[face.mIndices[j]]};
                const Eigen::Vector3f qNormal(aiNormal.x, aiNormal.y, aiNormal.z);
                switch (j)
                {
                case 0:
                    triangle.vertex_0.normal = qNormal;
                    break;
                case 1:
                    triangle.vertex_1.normal = qNormal;
                    break;
                case 2:
                    triangle.vertex_2.normal = qNormal;
                    break;
                default:;
                }
            }
        }
        else
        {
            triangle.loadNormal();
        }

        if (mesh->mColors[0] != nullptr)
        {
            for (unsigned int j = 0; j < 3; j++)
            {
                const aiColor4D &aiColor{mesh->mColors[0][face.mIndices[j]]};
                const Eigen::Vector4f qColor(aiColor.r, aiColor.g, aiColor.b, aiColor.a);
                switch (j)
                {
                case 0:
                    triangle.vertex_0.color = qColor;
                    break;
                case 1:
                    triangle.vertex_1.color = qColor;
                    break;
                case 2:
                    triangle.vertex_2.color = qColor;
                    break;
                default:;
                }
            }
        }
        else
        {
            triangle.vertex_0.color.reset();
            triangle.vertex_1.color.reset();
            triangle.vertex_2.color.reset();
        }

        model.m_triangleData.push_back(triangle);
    }
    m_modelData.push_back(model);
}

// std::tuple<size_t, size_t, std::vector<float>> normalizeImage(const std::string &imagePath)
// {
//     using namespace cv;
//     const Mat flipped_rgba{readImage(imagePath)};
//     std::vector<float> rgba{flipped_rgba.data, flipped_rgba.data + flipped_rgba.total() * flipped_rgba.elemSize()};
//     const int colsNum{flipped_rgba.cols};
//     const int rowsNum{flipped_rgba.rows};
//     return {colsNum, rowsNum, rgba};
// }
//
// cv::Mat readImage(const std::string &imageFilePath)
// {
//     using namespace cv;
//     // 禁用OpenCV并行处理，避免加载并行库失败的警告
//     setNumThreads(1);
//     setUseOptimized(false);
//     const Mat textureImg{imread(imageFilePath, IMREAD_UNCHANGED)};
//     if (textureImg.empty())
//     {
//         std::cout << "Error opening file " << imageFilePath;
//         return {};
//     }
//     Mat rgba_img; // 最终的 RGBA 格式图像
//     const int channels{textureImg.channels()};
//     if (channels == 4)
//     {
//         // 原始为 BGRA → 转换为 RGBA
//         cvtColor(textureImg, rgba_img, COLOR_BGRA2RGBA);
//     }
//     else if (channels == 3)
//     {
//         // 原始为 BGR → 转换为 RGBA（自动添加 Alpha=255）
//         cvtColor(textureImg, rgba_img, COLOR_BGR2RGBA);
//     }
//     else if (channels == 1)
//     {
//         // 原始为灰度图 → 转换为 RGBA（自动添加 Alpha=255）
//         cvtColor(textureImg, rgba_img, COLOR_GRAY2RGBA);
//     }
//     else
//     {
//         std::cout << "Error: 不支持的图像通道数！";
//         return {};
//     }
//     if (!rgba_img.isContinuous())
//     {
//         rgba_img = rgba_img.clone(); // 转为连续存储
//     }
//     Mat flipped_rgba;
//     flip(rgba_img, flipped_rgba, 0); // 0=垂直翻转（上下颠倒）
//     return flipped_rgba;
// }

void interpolation_helper(std::vector<float> &pointsWithColor, const Triangle &p, double accuracy, const std::vector<float> &rgba,
                          const size_t cols, const size_t rows)
{
    using vertex = Eigen::Vector3f;
    using texture = Eigen::Vector2f;

    if (pointsWithColor.size() >= 7)
    {
        const vertex baseVector3D_0{p.vector0()};
        const vertex baseVector3D_1{p.vector1()};
        texture baseVector2D_0{};
        texture baseVector2D_1{};
        if (p.vertex_0.texture.has_value() && p.vertex_1.texture.has_value() && p.vertex_2.texture.has_value())
        {
            baseVector2D_0 = p.vertex_1.texture.value() - p.vertex_0.texture.value();
            baseVector2D_1 = p.vertex_0.texture.value() - p.vertex_0.texture.value();
        }

        accuracy = (baseVector3D_0.norm() + baseVector3D_1.norm() / 2) / accuracy;

        const float acc0{static_cast<float>(1.0f / accuracy)};
        for (int i{0}; i < accuracy; ++i)
        {
            for (int j{0}; j < accuracy; ++j)
            {
                const auto temp{
                    p.vertex_0.vertex + acc0 * static_cast<float>(i) * (static_cast<float>(j) * acc0 * baseVector3D_0 + (1 - static_cast<float>(j) * acc0) * baseVector3D_1)};
                pointsWithColor.push_back(temp.x());
                pointsWithColor.push_back(temp.y());
                pointsWithColor.push_back(temp.z());
                texture texCoord;
                if (p.vertex_0.texture.has_value())
                {
                    texCoord = texture{
                        p.vertex_0.texture.value() + acc0 * static_cast<float>(i) * (static_cast<float>(j) * acc0 * baseVector2D_0 + (1 - static_cast<float>(j) * acc0) * baseVector2D_1)};
                }
                else
                {
                    std::cerr << "No Texture Data! Please generate it and try again! ";
                    break;
                }
                // 计算纹理坐标对应的像素位置（注意：纹理坐标x对应图片宽度，y对应图片高度）
                // 纹理坐标范围是[0, 1]，需要转换为像素坐标[0, cols-1]和[0, rows-1]
                const auto x_pixel{static_cast<size_t>(texCoord.x() * static_cast<float>(cols - 1))};
                const auto y_pixel{static_cast<size_t>(texCoord.y() * static_cast<float>(rows - 1))};
                // 计算RGBA数组中的索引（每个像素4个通道，行优先存储）
                const auto index = 4 * (y_pixel * cols + x_pixel);
                if (index + 4 < rgba.size())
                {
                    pointsWithColor.push_back(rgba[index] / 255.0f);
                    pointsWithColor.push_back(rgba[index + 1] / 255.0f);
                    pointsWithColor.push_back(rgba[index + 2] / 255.0f);
                    pointsWithColor.push_back(rgba[index + 3] / 255.0f);
                }
                else
                {
                    pointsWithColor.push_back(0.0f);
                    pointsWithColor.push_back(0.0f);
                    pointsWithColor.push_back(0.0f);
                    pointsWithColor.push_back(1.0f);
                }
            }
        }
    }
}

std::vector<float> &operator<<(std::vector<float> &lhs, const float rhs)
{
    lhs.push_back(rhs);
    return lhs;
}

// std::vector<float> outputVertexWithColorRGBAArray(const ModelData &modelsData, const char *imagePath, const double abs)
// {
//     auto [cols, rows, rgba]{normalizeImage(imagePath)};
//     std::vector<float> pointsWithColor;
//
//     for (const Triangle &p : modelsData.m_triangleData)
//     {
//         pointsWithColor << p.vertex_0.vertex.x() << p.vertex_0.vertex.y() << p.vertex_0.vertex.z() << p.vertex_1.vertex.x() << p.vertex_1.vertex.y() << p.vertex_1.vertex.z() << p.vertex_2.vertex.x() << p.vertex_2.vertex.y() << p.vertex_2.vertex.z();
//
//         if (p.vertex_0.texture.has_value() && p.vertex_1.texture.has_value() && p.vertex_2.texture.has_value())
//         {
//             const Eigen::Vector2f texCoord0{p.vertex_0.texture.value()};
//             const Eigen::Vector2f texCoord1{p.vertex_1.texture.value()};
//             const Eigen::Vector2f texCoord2{p.vertex_2.texture.value()};
//
//             // 计算纹理坐标对应的像素位置（注意：纹理坐标x对应图片宽度，y对应图片高度）
//             // 纹理坐标范围是[0, 1]，需要转换为像素坐标[0, cols-1]和[0, rows-1]
//             const auto x_pixel_0 = static_cast<size_t>(texCoord0.x() * static_cast<float>(cols - 1));
//             const auto y_pixel_0 = static_cast<size_t>(texCoord0.y() * static_cast<float>(rows - 1));
//             const auto x_pixel_1 = static_cast<size_t>(texCoord1.x() * static_cast<float>(cols - 1));
//             const auto y_pixel_1 = static_cast<size_t>(texCoord1.y() * static_cast<float>(rows - 1));
//             const auto x_pixel_2 = static_cast<size_t>(texCoord2.x() * static_cast<float>(cols - 1));
//             const auto y_pixel_2 = static_cast<size_t>(texCoord2.y() * static_cast<float>(rows - 1));
//
//             std::array<size_t, 3> xPixels{x_pixel_0, x_pixel_1, x_pixel_2};
//             std::array<size_t, 3> yPixels{y_pixel_0, y_pixel_1, y_pixel_2};
//
//             // 计算RGBA数组中的索引（每个像素4个通道，行优先存储）
//             for (short i{0}; i < 3; ++i)
//             {
//                 const auto index = 4 * (yPixels[i] * cols + xPixels[i]);
//                 if (index + 4 < rgba.size())
//                 {
//                     pointsWithColor.push_back(rgba[index] / 255.0f);
//                     pointsWithColor.push_back(rgba[index + 1] / 255.0f);
//                     pointsWithColor.push_back(rgba[index + 2] / 255.0f);
//                     pointsWithColor.push_back(rgba[index + 3] / 255.0f);
//                 }
//                 else
//                 {
//                     pointsWithColor.push_back(0.0f);
//                     pointsWithColor.push_back(0.0f);
//                     pointsWithColor.push_back(0.0f);
//                     pointsWithColor.push_back(1.0f);
//                 }
//             }
//         }
//         else
//         {
//             pointsWithColor.push_back(0.0f);
//             pointsWithColor.push_back(0.0f);
//             pointsWithColor.push_back(0.0f);
//             pointsWithColor.push_back(1.0f);
//         }
//
//         interpolation_helper(pointsWithColor, p, abs, rgba, cols, rows);
//     }
//     return pointsWithColor;
// }

void generateTextureCoords(
    const ModelData &modelsData,
    const Eigen::Vector3f &referencePlanePoint,
    const Eigen::Vector3f &referencePlaneNormal,
    const bool depthTest,
    bool z_bufferEnabled,
    const double range,
    const bool onlyOne)
{
    // 草，看不懂了。。。[裂开ing]

    // 给定参考平面，将参考平面转换称为平面 O-xy。
    const PlaneCoordinateTransform transformer{
        referencePlanePoint, referencePlaneNormal};
    ModelData newPlanes{modelsData};

    // 坐标变换，以参考平面和参考平面法向量，建立新的坐标系，简化计算
    for (Triangle &p : newPlanes.m_triangleData)
    {
        std::array<std::reference_wrapper<Eigen::Vector3f>, 3> vertices{p.vertex_0.vertex, p.vertex_1.vertex, p.vertex_2.vertex};
        std::array<std::reference_wrapper<Eigen::Vector3f>, 3> normals{p.vertex_0.normal, p.vertex_1.normal, p.vertex_2.normal};

        for (short i{0}; i < 3; ++i)
        {
            const Eigen::Vector3f tempVec{
                transformer.transformPoint(Eigen::Vector3f{
                    vertices[i].get().x(), vertices[i].get().y(), vertices[i].get().z()})};
            vertices[i].get() = {tempVec.x(), tempVec.y(), tempVec.z()};

            const Eigen::Vector3f tempNormalVec{
                transformer.transformVector(Eigen::Vector3f{normals[i].get().x(), normals[i].get().y(), normals[i].get().z()})};
            normals[i].get() = {tempNormalVec.x(), tempNormalVec.y(), tempNormalVec.z()};
        }
    }

    if (!newPlanes.m_triangleData.empty())
    {
        // 计算范围
        float maxX{newPlanes.m_triangleData[0].vertex_0.vertex.x()};
        float minX{newPlanes.m_triangleData[0].vertex_0.vertex.x()};
        float maxY{newPlanes.m_triangleData[0].vertex_0.vertex.y()};
        float minY{newPlanes.m_triangleData[0].vertex_0.vertex.y()};

        for (const Triangle &p : newPlanes.m_triangleData)
        {
            std::array<std::reference_wrapper<const Eigen::Vector3f>, 3> vertices{p.vertex_0.vertex, p.vertex_1.vertex, p.vertex_2.vertex};
            for (short i{0}; i < 3; ++i)
            {
                const auto valX{vertices[i].get().x()};
                const auto valY{vertices[i].get().y()};
                if (valX > maxX)
                {
                    maxX = valX;
                }
                else if (valX < minX)
                {
                    minX = valX;
                }
                if (valY > maxY)
                {
                    maxY = valY;
                }
                else if (valY < minY)
                {
                    minY = valY;
                }
            }
        }

        const Eigen::Vector3f referenceCenter{(maxX + minX) / 2, (maxY + minY) / 2, 0};

        // 根据给定 range 参数缩减贴图范围。
        maxX = static_cast<float>((maxX - referenceCenter.x()) * range + referenceCenter.x());
        minX = static_cast<float>((minX - referenceCenter.x()) * range + referenceCenter.x());
        maxY = static_cast<float>((maxY - referenceCenter.y()) * range + referenceCenter.y());
        minY = static_cast<float>((minY - referenceCenter.y()) * range + referenceCenter.y());

        // 生成纹理坐标
        const Eigen::Vector3f baseVert{minX, minY, 0.0f};

        for (size_t i{0}; i < modelsData.m_triangleData.size(); ++i)
        {
            bool to_generate{true};
            if (depthTest)
            {
                // 隐藏面剔除
                // 执行三角形转向判断，根据其 边0 和 边1 向量叉乘与投影平面正向法向量点乘的结果是否 ≤0 进行判断
                const Eigen::Vector3f vert0{newPlanes.m_triangleData[i].vertex_0.vertex};
                const Eigen::Vector3f vert1{newPlanes.m_triangleData[i].vertex_1.vertex};
                const Eigen::Vector3f vert2{newPlanes.m_triangleData[i].vertex_2.vertex};
                const auto bv0{vert1 - vert0};
                const auto bv1{vert2 - vert0};
                const Eigen::Vector3f triangleVec0{bv0};
                const Eigen::Vector3f triangleVec1{bv1};
                if (triangleVec0 != triangleVec1 && triangleVec0.norm() != 0 && triangleVec1.norm() != 0)
                {
                    const Eigen::Vector3f triangleCross{triangleVec0.cross(triangleVec1).normalized()};
                    const double dotResult{
                        triangleCross.dot(referencePlaneNormal)};
                    if (dotResult <= 0)
                    {
                        to_generate = false;
                    }
                }
                // 深度缓冲区计算
                // 使用模拟的 OpenGL 深度缓冲区 Z-Buffer 技术，目前暂时未实现。
            }
            if (to_generate)
            {
                std::array<std::reference_wrapper<Eigen::Vector3f>, 3> vertices{
                    newPlanes.m_triangleData[i].vertex_0.vertex, newPlanes.m_triangleData[i].vertex_1.vertex,
                    newPlanes.m_triangleData[i].vertex_2.vertex};
                std::array<std::reference_wrapper<std::optional<Eigen::Vector2f>>, 3> textures{
                    newPlanes.m_triangleData[i].vertex_0.texture, newPlanes.m_triangleData[i].vertex_1.texture, newPlanes.m_triangleData[i].vertex_2.texture};
                for (short j{0}; j < 3; ++j)
                {
                    const float textureX{vertices[j].get().x()};
                    const float textureY{vertices[j].get().y()};
                    if (!onlyOne || (textureX >= minX && textureX <= maxX && textureY >= minY && textureY <= maxY))
                    {
                        // 复制纹理坐标
                        textures[j].get() = {
                            (textureX - baseVert.x()) / (maxX - minX), (textureY - baseVert.y()) / (maxY - minY)};
                    }
                }
            }
        }
    }
}
