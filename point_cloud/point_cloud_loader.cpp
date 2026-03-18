//
// Created by jehor on 2026/3/4.
//

#include "point_cloud_loader.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <cstring>
#include <random>
#include <ranges>

struct EigenVector3iHash
{
    size_t operator()(const Eigen::Vector3f& v) const
    {
        size_t h1 = std::hash<int>()(v.x());
        size_t h2 = std::hash<int>()(v.y());
        size_t h3 = std::hash<int>()(v.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// 相等比较器
struct EigenVector3iEqual
{
    bool operator()(const Eigen::Vector3f& a, const Eigen::Vector3f& b) const
    {
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

// 定义哈希集合
using PointSet = std::unordered_set<Eigen::Vector3f, EigenVector3iHash, EigenVector3iEqual>;

PointCloudLoader::PointCloudLoader() = default;

void PointCloudLoader::generatePointCloud(const ModelData& modelData, double density)
{
    m_pointCloud.clear();
    for (const Triangle& triangle : modelData.m_triangleData) {
        if (triangle.vertex_0.color.has_value() && triangle.vertex_1.color.has_value() && triangle.vertex_2.color.
            has_value()) {
            interpolateTriangle(triangle, density);
        }
        else {
            std::cerr << "Point cloud has no triangle data" << std::endl;
            break;
        }
    }
}

void PointCloudLoader::binarizePointCloud(double density)
{
    // std::vector<Eigen::Vector3f> newPointCloud;

    const std::function translateToGray{
        [](float colorR, float colorG, float colorB)
        {
            return (colorR * 0.299f + colorG * 0.587f + colorB * 0.114f);
        }
    };

    if (m_pointCloud.empty()) {
        return;
    }

    const Eigen::Vector3f& firstPoint{m_pointCloud.at(0).vertex};
    float minX{firstPoint.x()};
    float minY{firstPoint.y()};
    float minZ{firstPoint.z()};
    float maxX{firstPoint.x()};
    float maxY{firstPoint.y()};
    float maxZ{firstPoint.z()};

    auto findPolarValue{
        [](float& min, float& max, const auto& ctn)
        {
            for (const float value : ctn) {
                if (min > value) {
                    min = value;
                }
                if (max < value) {
                    max = value;
                }
            }
        }
    };

    findPolarValue(minX, maxX, m_pointCloud | std::views::transform([](const BasicPoint& p) { return p.vertex.x(); }));
    findPolarValue(minY, maxY, m_pointCloud | std::views::transform([](const BasicPoint& p) { return p.vertex.y(); }));
    findPolarValue(minZ, maxZ, m_pointCloud | std::views::transform([](const BasicPoint& p) { return p.vertex.z(); }));

    auto binaryFunc{[](int x, int y, int z, size_t count) -> std::vector<Eigen::Vector3f> {
        std::vector<Eigen::Vector3f> points;
        points.reserve(count);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);
        for (size_t i = 0; i < count; ++i) {
            points.emplace_back(
                static_cast<float>(x) + dis(gen),
                static_cast<float>(y) + dis(gen),
                static_cast<float>(z) + dis(gen)
            );
        }
        return points;
    }};


    PointSet pointSet;
    for (const BasicPoint& point : m_pointCloud) {
        auto getGridPoint{
            [density](const Eigen::Vector3f& p)
            {
                const float gx{static_cast<float>(round(p.x() / density))};
                const float gy{static_cast<float>(round(p.y() / density))};
                const float gz{static_cast<float>(round(p.z() / density))};
                return Eigen::Vector3f{gx, gy, gz};
            }
        };

        Eigen::Vector3f tempPoint{getGridPoint(point.vertex)};
        const auto [iter, status]{pointSet.insert(tempPoint)};

        if (status) {
            const int gray {static_cast<int>(translateToGray(point.color.x(), point.color.y(), point.color.z()) * 10)};
            const auto p{iter};
            std::ranges::copy(binaryFunc(static_cast<int>(p->x()), static_cast<int>(p->y()), static_cast<int>(p->z()), gray), std::back_inserter(m_binaryPointCloud));
        }
    }


}

const std::vector<BasicPoint>& PointCloudLoader::getPointCloud() const
{
    return m_pointCloud;
}

void PointCloudLoader::clear()
{
    m_pointCloud.clear();
}

bool PointCloudLoader::exportToPLY(const std::string& filePath, const PLYFormat format) const
{
    std::ofstream file{filePath, std::ios::binary};
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return false;
    }

    const size_t vertexCount{m_pointCloud.size()};

    file << "ply\n";
    file << "format ";
    switch (format) {
    case PLYFormat::ASCII:
        file << "ascii 1.0\n";
        break;
    case PLYFormat::BinaryLittleEndian:
        file << "binary_little_endian 1.0\n";
        break;
    case PLYFormat::BinaryBigEndian:
        file << "binary_big_endian 1.0\n";
        break;
    }
    file << "element vertex " << vertexCount << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "property uchar alpha\n";
    file << "end_header\n";

    if (format == PLYFormat::ASCII) {
        for (const BasicPoint& point : m_pointCloud) {
            const unsigned char r{static_cast<unsigned char>(point.color.x() * 255.0f)};
            const unsigned char g{static_cast<unsigned char>(point.color.y() * 255.0f)};
            const unsigned char b{static_cast<unsigned char>(point.color.z() * 255.0f)};
            const unsigned char a{static_cast<unsigned char>(point.color.w() * 255.0f)};
            file << point.vertex.x() << " " << point.vertex.y() << " " << point.vertex.z() << " "
                << static_cast<int>(r) << " " << static_cast<int>(g) << " "
                << static_cast<int>(b) << " " << static_cast<int>(a) << "\n";
        }
    }
    else if (format == PLYFormat::BinaryLittleEndian) {
        for (const BasicPoint& point : m_pointCloud) {
            float x{point.vertex.x()};
            float y{point.vertex.y()};
            float z{point.vertex.z()};
            unsigned char r{static_cast<unsigned char>(point.color.x() * 255.0f)};
            unsigned char g{static_cast<unsigned char>(point.color.y() * 255.0f)};
            unsigned char b{static_cast<unsigned char>(point.color.z() * 255.0f)};
            unsigned char a{static_cast<unsigned char>(point.color.w() * 255.0f)};

            file.write(reinterpret_cast<const char*>(&x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            file.write(reinterpret_cast<const char*>(&r), sizeof(unsigned char));
            file.write(reinterpret_cast<const char*>(&g), sizeof(unsigned char));
            file.write(reinterpret_cast<const char*>(&b), sizeof(unsigned char));
            file.write(reinterpret_cast<const char*>(&a), sizeof(unsigned char));
        }
    }
    else {
        for (const BasicPoint& point : m_pointCloud) {
            float x{point.vertex.x()};
            float y{point.vertex.y()};
            float z{point.vertex.z()};
            unsigned char r{static_cast<unsigned char>(point.color.x() * 255.0f)};
            unsigned char g{static_cast<unsigned char>(point.color.y() * 255.0f)};
            unsigned char b{static_cast<unsigned char>(point.color.z() * 255.0f)};
            unsigned char a{static_cast<unsigned char>(point.color.w() * 255.0f)};

            auto writeFloatBE = [&file](float value)
            {
                const char* bytes{reinterpret_cast<const char*>(&value)};
                for (int i{3}; i >= 0; --i) {
                    file.write(&bytes[i], 1);
                }
            };

            writeFloatBE(x);
            writeFloatBE(y);
            writeFloatBE(z);
            file.write(reinterpret_cast<const char*>(&r), sizeof(unsigned char));
            file.write(reinterpret_cast<const char*>(&g), sizeof(unsigned char));
            file.write(reinterpret_cast<const char*>(&b), sizeof(unsigned char));
            file.write(reinterpret_cast<const char*>(&a), sizeof(unsigned char));
        }
    }

    file.close();
    return true;
}

bool PointCloudLoader::exportToDXF(const std::string& filePath, DXFFormat format) const
{
    std::ofstream file;
    if (format == DXFFormat::ASCII) {
        file.open(filePath);
    }
    else {
        file.open(filePath, std::ios::binary);
    }

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return false;
    }

    if (format == DXFFormat::ASCII) {
        // 写入 DXF 头部
        file << "0\nSECTION\n";
        file << "2\nHEADER\n";
        file << "9\n$ACADVER\n";
        file << "1\nAC1009\n";
        file << "0\nENDSEC\n";

        // 写入表部分
        file << "0\nSECTION\n";
        file << "2\nTABLES\n";
        file << "0\nTABLE\n";
        file << "2\nLAYER\n";
        file << "70\n1\n";
        file << "0\nLAYER\n";
        file << "2\n0\n";
        file << "70\n0\n";
        file << "62\n7\n";
        file << "6\nCONTINUOUS\n";
        file << "0\nENDTAB\n";
        file << "0\nENDSEC\n";

        // 写入块部分
        file << "0\nSECTION\n";
        file << "2\nBLOCKS\n";
        file << "0\nBLOCK\n";
        file << "2\n*Model_Space\n";
        file << "70\n0\n";
        file << "10\n0.0\n";
        file << "20\n0.0\n";
        file << "30\n0.0\n";
        file << "0\nENDBLK\n";
        file << "0\nENDSEC\n";

        // 写入实体部分
        file << "0\nSECTION\n";
        file << "2\nENTITIES\n";

        // 写入点云数据
        for (const BasicPoint& point : m_pointCloud) {
            file << "0\nPOINT\n";
            file << "8\n0\n";
            file << "10\n" << point.vertex.x() << "\n";
            file << "20\n" << point.vertex.y() << "\n";
            file << "30\n" << point.vertex.z() << "\n";
            // 写入颜色信息
            if (point.color.x() >= 0 && point.color.y() >= 0 && point.color.z() >= 0) {
                int colorIndex = static_cast<int>((point.color.x() + point.color.y() + point.color.z()) / 3 * 255);
                file << "62\n" << colorIndex << "\n";
            }
        }

        file << "0\nENDSEC\n";
        file << "0\nEOF\n";
    }
    else {
        // 二进制 DXF 格式
        // 写入 DXF 二进制文件头
        char header[130] = {0};
        header[0] = 0x00;
        header[1] = 0x00;
        header[2] = 0x41;
        header[3] = 0x43;
        header[4] = 0x31;
        header[5] = 0x30;
        header[6] = 0x30;
        header[7] = 0x39;
        file.write(header, 130);

        // 写入 SECTION 开始
        auto writeGroupCode = [&file, format](int code, const std::string& value)
        {
            // 写入组码
            short groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&groupCode), sizeof(short));
            }
            else {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入字符串值
            file.write(value.c_str(), value.size() + 1); // +1 包括 null 终止符
        };

        auto writeGroupCodeDouble = [&file, format](int code, double value)
        {
            // 写入组码
            short groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&groupCode), sizeof(short));
            }
            else {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入双精度值
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&value), sizeof(double));
            }
            else {
                // 大端序
                char bytes[8];
                const char* originalBytes = reinterpret_cast<const char*>(&value);
                for (int i = 0; i < 8; ++i) {
                    bytes[i] = originalBytes[7 - i];
                }
                file.write(bytes, 8);
            }
        };

        auto writeGroupCodeInt = [&file, format](int code, int value)
        {
            // 写入组码
            short groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&groupCode), sizeof(short));
            }
            else {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入整数值
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&value), sizeof(int));
            }
            else {
                // 大端序
                char bytes[4];
                const char* originalBytes = reinterpret_cast<const char*>(&value);
                for (int i = 0; i < 4; ++i) {
                    bytes[i] = originalBytes[3 - i];
                }
                file.write(bytes, 4);
            }
        };

        // 写入头部
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "HEADER");
        writeGroupCode(9, "$ACADVER");
        writeGroupCode(1, "AC1009");
        writeGroupCode(0, "ENDSEC");

        // 写入表部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "TABLES");
        writeGroupCode(0, "TABLE");
        writeGroupCode(2, "LAYER");
        writeGroupCodeInt(70, 1);
        writeGroupCode(0, "LAYER");
        writeGroupCode(2, "0");
        writeGroupCodeInt(70, 0);
        writeGroupCodeInt(62, 7);
        writeGroupCode(6, "CONTINUOUS");
        writeGroupCode(0, "ENDTAB");
        writeGroupCode(0, "ENDSEC");

        // 写入块部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "BLOCKS");
        writeGroupCode(0, "BLOCK");
        writeGroupCode(2, "*Model_Space");
        writeGroupCodeInt(70, 0);
        writeGroupCodeDouble(10, 0.0);
        writeGroupCodeDouble(20, 0.0);
        writeGroupCodeDouble(30, 0.0);
        writeGroupCode(0, "ENDBLK");
        writeGroupCode(0, "ENDSEC");

        // 写入实体部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "ENTITIES");

        // 写入点云数据
        for (const BasicPoint& point : m_pointCloud) {
            writeGroupCode(0, "POINT");
            writeGroupCode(8, "0");
            writeGroupCodeDouble(10, point.vertex.x());
            writeGroupCodeDouble(20, point.vertex.y());
            writeGroupCodeDouble(30, point.vertex.z());
            // 写入颜色信息
            if (point.color.x() >= 0 && point.color.y() >= 0 && point.color.z() >= 0) {
                int colorIndex = static_cast<int>((point.color.x() + point.color.y() + point.color.z()) / 3 * 255);
                writeGroupCodeInt(62, colorIndex);
            }
        }

        writeGroupCode(0, "ENDSEC");
        writeGroupCode(0, "EOF");
    }

    file.close();
    return true;
}

bool PointCloudLoader::exportBinaryToDXF(const std::string& filePath, DXFFormat format) const
{
    std::ofstream file;
    if (format == DXFFormat::ASCII) {
        file.open(filePath);
    }
    else {
        file.open(filePath, std::ios::binary);
    }

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return false;
    }

    if (format == DXFFormat::ASCII) {
        // 写入 DXF 头部
        file << "0\nSECTION\n";
        file << "2\nHEADER\n";
        file << "9\n$ACADVER\n";
        file << "1\nAC1009\n";
        file << "0\nENDSEC\n";

        // 写入表部分
        file << "0\nSECTION\n";
        file << "2\nTABLES\n";
        file << "0\nTABLE\n";
        file << "2\nLAYER\n";
        file << "70\n1\n";
        file << "0\nLAYER\n";
        file << "2\n0\n";
        file << "70\n0\n";
        file << "62\n7\n";
        file << "6\nCONTINUOUS\n";
        file << "0\nENDTAB\n";
        file << "0\nENDSEC\n";

        // 写入块部分
        file << "0\nSECTION\n";
        file << "2\nBLOCKS\n";
        file << "0\nBLOCK\n";
        file << "2\n*Model_Space\n";
        file << "70\n0\n";
        file << "10\n0.0\n";
        file << "20\n0.0\n";
        file << "30\n0.0\n";
        file << "0\nENDBLK\n";
        file << "0\nENDSEC\n";

        // 写入实体部分
        file << "0\nSECTION\n";
        file << "2\nENTITIES\n";

        // 写入点云数据
        for (const Eigen::Vector3f& point : m_binaryPointCloud) {
            file << "0\nPOINT\n";
            file << "8\n0\n";
            file << "10\n" << point.x() << "\n";
            file << "20\n" << point.y() << "\n";
            file << "30\n" << point.z() << "\n";
            file << "62\n" << 0 << "\n";

        }

        file << "0\nENDSEC\n";
        file << "0\nEOF\n";
    }
    else {
        // 二进制 DXF 格式
        // 写入 DXF 二进制文件头
        char header[130] = {0};
        header[0] = 0x00;
        header[1] = 0x00;
        header[2] = 0x41;
        header[3] = 0x43;
        header[4] = 0x31;
        header[5] = 0x30;
        header[6] = 0x30;
        header[7] = 0x39;
        file.write(header, 130);

        // 写入 SECTION 开始
        auto writeGroupCode = [&file, format](int code, const std::string& value)
        {
            // 写入组码
            short groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&groupCode), sizeof(short));
            }
            else {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入字符串值
            file.write(value.c_str(), value.size() + 1); // +1 包括 null 终止符
        };

        auto writeGroupCodeDouble = [&file, format](int code, double value)
        {
            // 写入组码
            short groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&groupCode), sizeof(short));
            }
            else {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入双精度值
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&value), sizeof(double));
            }
            else {
                // 大端序
                char bytes[8];
                const char* originalBytes = reinterpret_cast<const char*>(&value);
                for (int i = 0; i < 8; ++i) {
                    bytes[i] = originalBytes[7 - i];
                }
                file.write(bytes, 8);
            }
        };

        auto writeGroupCodeInt = [&file, format](int code, int value)
        {
            // 写入组码
            short groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&groupCode), sizeof(short));
            }
            else {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入整数值
            if (format == DXFFormat::BinaryLittleEndian) {
                file.write(reinterpret_cast<const char*>(&value), sizeof(int));
            }
            else {
                // 大端序
                char bytes[4];
                const char* originalBytes = reinterpret_cast<const char*>(&value);
                for (int i = 0; i < 4; ++i) {
                    bytes[i] = originalBytes[3 - i];
                }
                file.write(bytes, 4);
            }
        };

        // 写入头部
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "HEADER");
        writeGroupCode(9, "$ACADVER");
        writeGroupCode(1, "AC1009");
        writeGroupCode(0, "ENDSEC");

        // 写入表部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "TABLES");
        writeGroupCode(0, "TABLE");
        writeGroupCode(2, "LAYER");
        writeGroupCodeInt(70, 1);
        writeGroupCode(0, "LAYER");
        writeGroupCode(2, "0");
        writeGroupCodeInt(70, 0);
        writeGroupCodeInt(62, 7);
        writeGroupCode(6, "CONTINUOUS");
        writeGroupCode(0, "ENDTAB");
        writeGroupCode(0, "ENDSEC");

        // 写入块部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "BLOCKS");
        writeGroupCode(0, "BLOCK");
        writeGroupCode(2, "*Model_Space");
        writeGroupCodeInt(70, 0);
        writeGroupCodeDouble(10, 0.0);
        writeGroupCodeDouble(20, 0.0);
        writeGroupCodeDouble(30, 0.0);
        writeGroupCode(0, "ENDBLK");
        writeGroupCode(0, "ENDSEC");

        // 写入实体部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "ENTITIES");

        // 写入点云数据
        for (const Eigen::Vector3f& point : m_binaryPointCloud) {
            writeGroupCode(0, "POINT");
            writeGroupCode(8, "0");
            writeGroupCodeDouble(10, point.x());
            writeGroupCodeDouble(20, point.y());
            writeGroupCodeDouble(30, point.z());
            writeGroupCodeInt(62, 0);

        }

        writeGroupCode(0, "ENDSEC");
        writeGroupCode(0, "EOF");
    }

    file.close();
    return true;
}

OriginalModel PointCloudLoader::getAllModelData() const
{
    return m_pointCloud;
}

const OriginalModel& PointCloudLoader::getAllModelDataR() const
{
    return m_pointCloud;
}

void PointCloudLoader::interpolateTriangle(const Triangle& triangle, double density)
{
    const Eigen::Vector3f v0{triangle.vertex_0.vertex};
    const Eigen::Vector3f v1{triangle.vertex_1.vertex};
    const Eigen::Vector3f v2{triangle.vertex_2.vertex};

    const Eigen::Vector4f c0{triangle.vertex_0.color.value()};
    const Eigen::Vector4f c1{triangle.vertex_1.color.value()};
    const Eigen::Vector4f c2{triangle.vertex_2.color.value()};

    const Eigen::Vector3f edge0{v1 - v0};
    const Eigen::Vector3f edge1{v2 - v0};
    const double edge0Length{edge0.norm()};
    const double edge1Length{edge1.norm()};

    const int numPointsU{static_cast<int>(std::ceil(edge0Length / density))};
    const int numPointsV{static_cast<int>(std::ceil(edge1Length / density))};

    for (int i{0}; i <= numPointsU; ++i) {
        for (int j{0}; j <= numPointsV - i; ++j) {
            const double u{static_cast<double>(i) / static_cast<double>(numPointsU)};
            const double v{static_cast<double>(j) / static_cast<double>(numPointsV)};
            const double w{1.0 - u - v};

            if (w >= 0.0) {
                const Eigen::Vector3f position{u * v1 + v * v2 + w * v0};
                const Eigen::Vector4f color{u * c1 + v * c2 + w * c0};

                BasicPoint point{};
                point.vertex = position;
                point.color = color;
                m_pointCloud.push_back(point);
            }
        }
    }
}
