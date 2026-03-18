//
// Created by jehor on 2026/3/18.
//

#include "BinaryMesh.h"
#include <random>
#include <unordered_set>
#include <fstream>
#include <iostream>

int &BasicMesh::get(size_t x, size_t y, size_t z)
{
    // 检查坐标范围，防止数组越界
    if (x >= 4 || y >= 4 || z >= 4)
    {
        std::cerr << "Error: BasicMesh::get() - Invalid coordinates: "
                  << "x=" << x << ", y=" << y << ", z=" << z << std::endl;
        throw std::out_of_range("BasicMesh coordinates out of range");
    }

    // 正确计算索引：z * 16 + y * 4 + x
    return meshData.at(z * 16 + y * 4 + x);
}

BinaryMesh::BinaryMesh(const OriginalModel &data, double resolution)
{
    if (data.empty())
        return;

    // 获取最值，用于确定盒子范围
    const Eigen::Vector3f &firstPoint{data.at(0).vertex};
    float minX{firstPoint.x()};
    float minY{firstPoint.y()};
    float minZ{firstPoint.z()};
    float maxX{firstPoint.x()};
    float maxY{firstPoint.y()};
    float maxZ{firstPoint.z()};

    auto findPolarValue{
        [](float &min, float &max, const auto &ctn)
        {
            for (const float value : ctn)
            {
                if (min > value)
                {
                    min = value;
                }
                if (max < value)
                {
                    max = value;
                }
            }
        }};

    // 使用视图获取最值
    findPolarValue(minX, maxX, data | std::views::transform([](const BasicPoint &p)
                                                            { return p.vertex.x(); }));
    findPolarValue(minY, maxY, data | std::views::transform([](const BasicPoint &p)
                                                            { return p.vertex.y(); }));
    findPolarValue(minZ, maxZ, data | std::views::transform([](const BasicPoint &p)
                                                            { return p.vertex.z(); }));

    // 获取最值完毕

    const long long xRange{static_cast<long long>((maxX - minX) / resolution)};
    const long long yRange{static_cast<long long>((maxY - minY) / resolution)};
    const long long zRange{static_cast<long long>((maxZ - minZ) / resolution)};

    // 检查网格大小，防止内存分配失败
    const long long max_dimension = 1000000; // 设置最大维度限制
    if (xRange > max_dimension || yRange > max_dimension || zRange > max_dimension)
    {
        std::cerr << "Error: Mesh dimensions too large, memory allocation would fail." << std::endl;
        std::cerr << "xRange: " << xRange << ", yRange: " << yRange << ", zRange: " << zRange << std::endl;
        std::cerr << "Please increase resolution or process smaller model." << std::endl;
        // 初始化空张量，防止后续访问时出错
        m_originalMesh = Eigen::Tensor<int, 3>(0, 0, 0);
        m_processingMesh = Eigen::Tensor<BasicMesh, 3>(0, 0, 0);
        return;
    }

    // 检查总元素数量，防止内存分配失败
    const long long total_elements = static_cast<long long>(xRange) * yRange * zRange;
    const long long max_elements = 1000000000; // 设置最大元素数量限制
    if (total_elements > max_elements)
    {
        std::cerr << "Error: Total mesh elements too large, memory allocation would fail." << std::endl;
        std::cerr << "Total elements: " << total_elements << ", Max allowed: " << max_elements << std::endl;
        std::cerr << "Please increase resolution or process smaller model." << std::endl;
        // 初始化空张量，防止后续访问时出错
        m_originalMesh = Eigen::Tensor<int, 3>(0, 0, 0);
        m_processingMesh = Eigen::Tensor<BasicMesh, 3>(0, 0, 0);
        return;
    }

    m_originalMesh = Eigen::Tensor<int, 3>(xRange, yRange, zRange); // 初始化盒子
    m_processingMesh = Eigen::Tensor<BasicMesh, 3>(xRange, yRange, zRange);
    m_originalMesh.setConstant(-1);

    // 加载盒子网格数据
    for (const BasicPoint &point : data)
    {
        const long long xPosition{static_cast<long long>((point.vertex.x() - minX) / resolution)};
        const long long yPosition{static_cast<long long>((point.vertex.y() - minY) / resolution)};
        const long long zPosition{static_cast<long long>((point.vertex.z() - minZ) / resolution)};
        int &pos{m_originalMesh(xPosition, yPosition, zPosition)};
        if (pos == -1)
        {
            pos = toGray(point.color);
            // std::cout << pos << std::endl;
        }
    }
}

void BinaryMesh::process()
{
    std::random_device m_rd{};
    std::mt19937 m_generator{m_rd()};
    std::uniform_int_distribution<int> m_distribution{0, 63};

    for (long long dimensionX{0}; dimensionX < m_originalMesh.dimension(0); ++dimensionX)
    {
        for (long long dimensionY{0}; dimensionY < m_originalMesh.dimension(1); ++dimensionY)
        {
            for (long long dimensionZ{0}; dimensionZ < m_originalMesh.dimension(2); ++dimensionZ)
            {
                const int stair{m_originalMesh(dimensionX, dimensionY, dimensionZ) / 4};
                if (stair <= 0)
                {
                    continue;
                }

                // 检查stair值，防止创建过大的集合
                const int max_stair = 64; // BasicMesh::data的大小是64
                int actual_stair = stair;
                if (actual_stair > max_stair)
                {
                    std::cerr << "Warning: Stair value too large, clamping to max allowed." << std::endl;
                    std::cerr << "Stair: " << actual_stair << ", Max allowed: " << max_stair << std::endl;
                    actual_stair = max_stair;
                }

                std::unordered_set<int> nums;

                while (nums.size() < actual_stair)
                {
                    int num = m_distribution(m_generator);
                    nums.insert(num);
                }
                std::vector<int> result(nums.begin(), nums.end());
                for (const int i : result)
                {
                    m_processingMesh(dimensionX, dimensionY, dimensionZ).meshData.at(i) = 1;
                }
            }
        }
    }
}

bool BinaryMesh::outputDXF(const std::string &fileName, DXFFormat format)
{
    std::ofstream file;
    if (format == DXFFormat::ASCII)
    {
        file.open(fileName);
    }
    else
    {
        file.open(fileName, std::ios::binary);
    }

    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return false;
    }

    if (format == DXFFormat::ASCII)
    {
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
        const double step = 1.0; // 固定步长
        for (long long x = 0; x < m_processingMesh.dimension(0); ++x)
        {
            for (long long y = 0; y < m_processingMesh.dimension(1); ++y)
            {
                for (long long z = 0; z < m_processingMesh.dimension(2); ++z)
                {
                    const BasicMesh &mesh = m_processingMesh(x, y, z);
                    for (int i = 0; i < 64; ++i)
                    {
                        if (mesh.meshData[i] == 1)
                        {
                            // 将 64 个索引转换为 4x4x4 坐标
                            int subX = i % 4;
                            int subY = (i / 4) % 4;
                            int subZ = i / 16;

                            // 计算最终坐标
                            const auto finalX = static_cast<float>(static_cast<double>(x) * 4 + subX * step);
                            const auto finalY = static_cast<float>(static_cast<double>(y) * 4 + subY * step);
                            const auto finalZ = static_cast<float>(static_cast<double>(z) * 4 + subZ * step);

                            // 写入点
                            file << "0\nPOINT\n";
                            file << "8\n0\n";
                            file << "10\n"
                                 << finalX << "\n";
                            file << "20\n"
                                 << finalY << "\n";
                            file << "30\n"
                                 << finalZ << "\n";
                        }
                    }
                }
            }
        }

        file << "0\nENDSEC\n";
        file << "0\nEOF\n";
    }
    else
    {
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
        auto writeGroupCode = [&file, format](const int code, const std::string &value)
        {
            // 写入组码
            auto groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian)
            {
                file.write(reinterpret_cast<const char *>(&groupCode), sizeof(short));
            }
            else
            {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入字符串值
            file.write(value.c_str(), static_cast<long long>(value.size()) + 1); // +1 for null terminator
        };

        auto writeDouble = [&file, format](const int code, const double value)
        {
            // 写入组码
            auto groupCode = static_cast<short>(code);
            if (format == DXFFormat::BinaryLittleEndian)
            {
                file.write(reinterpret_cast<const char *>(&groupCode), sizeof(short));
            }
            else
            {
                // 大端序
                char bytes[2];
                bytes[0] = static_cast<char>((groupCode >> 8) & 0xFF);
                bytes[1] = static_cast<char>(groupCode & 0xFF);
                file.write(bytes, 2);
            }
            // 写入双精度浮点数
            if (format == DXFFormat::BinaryLittleEndian)
            {
                file.write(reinterpret_cast<const char *>(&value), sizeof(double));
            }
            else
            {
                // 大端序
                char bytes[8];
                double temp = value;
                char *src = reinterpret_cast<char *>(&temp);
                for (int i = 0; i < 8; ++i)
                {
                    bytes[7 - i] = src[i];
                }
                file.write(bytes, 8);
            }
        };

        // 写入头部部分
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
        writeGroupCode(70, "1");
        writeGroupCode(0, "LAYER");
        writeGroupCode(2, "0");
        writeGroupCode(70, "0");
        writeGroupCode(62, "7");
        writeGroupCode(6, "CONTINUOUS");
        writeGroupCode(0, "ENDTAB");
        writeGroupCode(0, "ENDSEC");

        // 写入块部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "BLOCKS");
        writeGroupCode(0, "BLOCK");
        writeGroupCode(2, "*Model_Space");
        writeGroupCode(70, "0");
        writeDouble(10, 0.0);
        writeDouble(20, 0.0);
        writeDouble(30, 0.0);
        writeGroupCode(0, "ENDBLK");
        writeGroupCode(0, "ENDSEC");

        // 写入实体部分
        writeGroupCode(0, "SECTION");
        writeGroupCode(2, "ENTITIES");

        // 写入点云数据
        const double step = 1.0; // 固定步长
        for (long long x = 0; x < m_processingMesh.dimension(0); ++x)
        {
            for (long long y = 0; y < m_processingMesh.dimension(1); ++y)
            {
                for (long long z = 0; z < m_processingMesh.dimension(2); ++z)
                {
                    const BasicMesh &mesh = m_processingMesh(x, y, z);
                    for (int i = 0; i < 64; ++i)
                    {
                        if (mesh.meshData[i] == 1)
                        {
                            // 将 64 个索引转换为 4x4x4 坐标
                            int subX = i % 4;
                            int subY = (i / 4) % 4;
                            int subZ = i / 16;

                            // 计算最终坐标
                            double finalX = static_cast<double>(x * 4 + subX) * step;
                            double finalY = static_cast<double>(y * 4 + subY) * step;
                            double finalZ = static_cast<double>(z * 4 + subZ) * step;

                            // 写入点
                            writeGroupCode(0, "POINT");
                            writeGroupCode(8, "0");
                            writeDouble(10, finalX);
                            writeDouble(20, finalY);
                            writeDouble(30, finalZ);
                        }
                    }
                }
            }
        }

        writeGroupCode(0, "ENDSEC");
        writeGroupCode(0, "EOF");
    }

    file.close();
    return true;
}

int BinaryMesh::toGray(const Eigen::Vector4f &normalizeColor)
{
    const float &colorR{normalizeColor.x()};
    const float &colorG{normalizeColor.y()};
    const float &colorB{normalizeColor.z()};

    return static_cast<int>((colorR * 0.299f + colorG * 0.587f + colorB * 0.114f) * 255);
}
