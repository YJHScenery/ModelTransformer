#include <iostream>
#include <assimp/Importer.hpp>
#include "model_loader/model_loader.h"
#include "point_cloud/point_cloud_loader.h"
#include "CLI11.hpp"
#include "BinaryMesh.h"

int main(int argc, char* argv[])
{
    try {
        CLI::App app{"Model Transformer - Convert OBJ model to point cloud"};

        std::string inputFile;
        std::string outputFile;
        double density = 0.1;
        double resolution = 5;
        std::string format_string;
        std::string output_type = "ply";

        const std::map<std::string, PLYFormat> PLYFormatMap{
            {"ascii", PLYFormat::ASCII},
            {"ASCII", PLYFormat::ASCII},
            {"ble", PLYFormat::BinaryLittleEndian},
            {"BLE", PLYFormat::BinaryLittleEndian},
            {"bbe", PLYFormat::BinaryBigEndian},
            {"BBE", PLYFormat::BinaryBigEndian}
        };

        const std::map<std::string, DXFFormat> DXFFormatMap{
            {"ascii", DXFFormat::ASCII},
            {"ASCII", DXFFormat::ASCII},
            {"ble", DXFFormat::BinaryLittleEndian},
            {"BLE", DXFFormat::BinaryLittleEndian},
            {"bbe", DXFFormat::BinaryBigEndian},
            {"BBE", DXFFormat::BinaryBigEndian}
        };

        app.add_option("-i,--input", inputFile, "Input OBJ file path")
           ->required()
           ->check(CLI::ExistingFile);

        app.add_option("-d,--density", density, "Density parameter for point cloud generation")
           ->default_val(1)
           ->check(CLI::PositiveNumber);

        app.add_option("-o,--output", outputFile, "Output file path")
           ->required();

        app.add_option("-t,--type", output_type, "Output file type: \nply; \ndxf; \n 二值化 DXF")
           ->default_val("ply")
           ->check(CLI::IsMember({"ply", "dxf", "bdxf"}));

        app.add_option("-f, --format", format_string,
                       "File format: \nascii(ASCII); \nble(BinaryLittleEndian); \nbbe(BinaryBigEndian)")
           ->default_val("bbe");

        app.add_option("-r, --resolution", resolution, "Resolution parameter for point cloud generation")->
            default_val(5);

        CLI11_PARSE(app, argc, argv);

        ModelLoader loader;
        if (!loader.loadModelWithVertexColors(inputFile)) {
            std::cerr << "Error: Failed to load model from " << inputFile << std::endl;
            return 1;
        }

        std::cout << "Model loaded successfully!" << std::endl;

        const auto data{loader.getModelData()};

        PointCloudLoader pcl{};
        pcl.generatePointCloud(data, density);

        std::cout << "Point cloud generated with density: " << density << std::endl;

        bool exportSuccess = false;

        if (output_type == "ply") {
            if (!PLYFormatMap.contains(format_string)) {
                format_string = "bbe";
            }

            exportSuccess = pcl.exportToPLY(outputFile, PLYFormatMap.at(format_string));
        }
        else if (output_type == "dxf") {
            if (!DXFFormatMap.contains(format_string)) {
                format_string = "bbe";
            }

            exportSuccess = pcl.exportToDXF(outputFile, DXFFormatMap.at(format_string));
        }
        else if (output_type == "bdxf") {
            if (!DXFFormatMap.contains(format_string)) {
                format_string = "bbe";
            }

            OriginalModel modelData{pcl.getAllModelData()};
            BinaryMesh processor{modelData, resolution};
            processor.process();

            exportSuccess = processor.outputDXF(outputFile, DXFFormatMap.at(format_string));
        }

        if (exportSuccess) {
            std::cout << "Point cloud exported successfully to: " << outputFile << std::endl;
        }
        else {
            std::cerr << "Error: Failed to export point cloud to " << outputFile << std::endl;
            return 1;
        }
    }
    catch (std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    return 0;
}
