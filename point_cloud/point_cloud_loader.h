//
// Created by jehor on 2026/3/4.
//

#ifndef MODELTRANSFORMER_POINTCLOUDLOADER_H
#define MODELTRANSFORMER_POINTCLOUDLOADER_H

#include "model_loader.h"


class PointCloudLoader
{
public:
    PointCloudLoader();

    void generatePointCloud(const ModelData &modelData, double density);

    void binarizePointCloud(double density);

    [[nodiscard]] const std::vector<BasicPoint> &getPointCloud() const;

    void clear();

    [[nodiscard]] bool exportToPLY(const std::string &filePath, PLYFormat format = PLYFormat::ASCII) const;

    [[nodiscard]] bool exportToDXF(const std::string &filePath, DXFFormat format = DXFFormat::ASCII) const;

    [[nodiscard]] bool exportBinaryToDXF(const std::string &filePath, DXFFormat format = DXFFormat::ASCII) const;\

    [[nodiscard]] OriginalModel getAllModelData() const;

    [[nodiscard]] const OriginalModel& getAllModelDataR() const;

private:
    std::vector<BasicPoint> m_pointCloud;

    std::vector<Eigen::Vector3f> m_binaryPointCloud;

    void interpolateTriangle(const Triangle &triangle, double density);
};

#endif // MODELTRANSFORMER_POINTCLOUDLOADER_H