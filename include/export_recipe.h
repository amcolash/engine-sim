#ifndef ATG_ENGINE_SIM_EXPORT_RECIPE_H
#define ATG_ENGINE_SIM_EXPORT_RECIPE_H

#include <string>
#include <vector>
#include <cstdint>

struct ExportProfile {
    int rpmMin = 1000;
    int rpmMax = 6000;
    int rpmStep = 500;
    int cyclesPerLoop = 6;
    bool exportSteadyRpm = true;
    bool exportStarter = true;
    bool exportRevLimiter = true;
    bool exportRevBlip = true;
    bool exportDecelCrackle = true;

    // Custom list of RPMs if explicitly specified
    std::vector<int> explicitRpms;
};

struct VehicleExportConfig {
    std::string id;
    std::string displayName;
    std::string scriptPath;
    std::vector<std::string> extraSearchPaths;
    ExportProfile exportProfile;
};

struct GlobalExportSettings {
    std::string outputDir = "assets/audio/engines";
    int sampleRate = 44100;
    int bitsPerSample = 16;
    float normalizePeakDbfs = -1.0f;
    bool embedLoopMarkers = true;
    int defaultCyclesPerLoop = 6;
};

struct ExportRecipe {
    GlobalExportSettings globalSettings;
    std::vector<std::string> defaultSearchPaths;
    std::vector<VehicleExportConfig> vehicles;

    static bool loadFromFile(const std::string &filePath, ExportRecipe &recipe, std::string &errorMessage);
    static bool parseJsonString(const std::string &jsonStr, ExportRecipe &recipe, std::string &errorMessage);
    static ExportRecipe createDefaultSingleEngine(const std::string &scriptPath, const std::string &outputDir, int rpmStep = 500);
};

#endif /* ATG_ENGINE_SIM_EXPORT_RECIPE_H */

