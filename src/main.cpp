#include "../include/engine_sim_application.h"
#include "../include/audio_exporter.h"
#include "../include/export_recipe.h"
#include "build_info.h"

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>

static void printHelp(const char *executableName) {
    std::cout << "Engine Simulator (v" << ENGINE_SIM_PROJECT_VERSION << ")\n\n"
              << "USAGE:\n"
              << "  " << executableName << " [options]\n\n"
              << "INTERACTIVE GUI MODE:\n"
              << "  " << executableName << "                  Launch full interactive simulator\n\n"
              << "HEADLESS AUDIO EXPORTER MODE:\n"
              << "  --export-audio <recipe.json>     Run batch audio export using JSON recipe\n"
              << "  --export-sfx <type> [recipe]     Export specific SFX only (decel_crackle, engine_start, rev_blip, rev_limiter, all)\n"
              << "  --sfx <type>                     Filter export to specific SFX types (skips steady RPM loops)\n"
              << "  --vehicle <id>                   Filter export to a specific vehicle ID\n"
              << "  --export-engine <script.mr>      Quickly export a single engine script\n"
              << "      --out <dir>                  Output destination directory (default: assets/audio/engines)\n"
              << "      --rpm-step <step>            RPM interval step (default: 500)\n\n"
              << "OTHER OPTIONS:\n"
              << "  -h, --help                       Show this help message\n";
}

static std::string findDataRoot(const char *argv0) {
    std::string candidate = ENGINE_SIM_DATA_ROOT;
    if (std::filesystem::exists(candidate + "/assets")) {
        return candidate;
    }

    if (std::filesystem::exists("assets")) {
        return ".";
    }

    if (std::filesystem::exists("../assets")) {
        return "..";
    }

    try {
        std::filesystem::path exePath = std::filesystem::canonical(argv0).parent_path();
        if (std::filesystem::exists(exePath / "assets")) return exePath.string();
        if (std::filesystem::exists(exePath / "../assets")) return (exePath / "..").string();
        if (std::filesystem::exists(exePath / "../share/engine-sim/assets")) return (exePath / "../share/engine-sim").string();
    } catch (...) {}

    return ".";
}

static void runApp(void *handle, ysContextObject::DeviceAPI api) {
    EngineSimApplication application;
    application.initialize(handle, api);
    application.run();
    application.destroy();
}

int main(int argc, char *argv[]) {
    std::string exportRecipePath;
    std::string exportSfxType;
    std::string vehicleFilter;
    std::string singleEngineScript;
    std::string outputDir = "assets/audio/engines";
    int rpmStep = 500;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--export-audio" && i + 1 < argc) {
            exportRecipePath = argv[++i];
        } else if (arg == "--export-sfx" && i + 1 < argc) {
            exportSfxType = argv[++i];
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                std::string nextArg = argv[i + 1];
                if (nextArg.size() >= 5 && nextArg.substr(nextArg.size() - 5) == ".json") {
                    exportRecipePath = argv[++i];
                }
            }
        } else if (arg == "--sfx" && i + 1 < argc) {
            exportSfxType = argv[++i];
        } else if (arg == "--vehicle" && i + 1 < argc) {
            vehicleFilter = argv[++i];
        } else if (arg == "--export-engine" && i + 1 < argc) {
            singleEngineScript = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            outputDir = argv[++i];
        } else if (arg == "--rpm-step" && i + 1 < argc) {
            rpmStep = std::stoi(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            printHelp(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printHelp(argv[0]);
            return 1;
        }
    }

    // Default to game_vehicles.json if exporting SFX without explicit recipe
    if (exportRecipePath.empty() && !exportSfxType.empty()) {
        exportRecipePath = "recipes/game_vehicles.json";
    }

    // Headless batch export mode
    if (!exportRecipePath.empty()) {
        ExportRecipe recipe;
        std::string error;
        if (!ExportRecipe::loadFromFile(exportRecipePath, recipe, error)) {
            std::cerr << "Error loading recipe: " << error << std::endl;
            return 1;
        }

        if (!exportSfxType.empty()) {
            std::string sfxError;
            if (!recipe.applySfxFilter(exportSfxType, sfxError)) {
                std::cerr << "Error: " << sfxError << std::endl;
                return 1;
            }
        }

        if (!vehicleFilter.empty()) {
            if (!recipe.filterVehicles(vehicleFilter)) {
                std::cerr << "Error: Vehicle '" << vehicleFilter << "' not found in recipe " << exportRecipePath << std::endl;
                return 1;
            }
        }

        std::string dataRoot = findDataRoot(argv[0]);
        AudioExporter exporter;
        bool success = exporter.exportRecipe(recipe, dataRoot);
        return success ? 0 : 1;
    }

    // Single engine quick export mode
    if (!singleEngineScript.empty()) {
        ExportRecipe recipe = ExportRecipe::createDefaultSingleEngine(singleEngineScript, outputDir, rpmStep);
        if (!exportSfxType.empty()) {
            std::string sfxError;
            if (!recipe.applySfxFilter(exportSfxType, sfxError)) {
                std::cerr << "Error: " << sfxError << std::endl;
                return 1;
            }
        }
        std::string dataRoot = findDataRoot(argv[0]);
        AudioExporter exporter;
        bool success = exporter.exportRecipe(recipe, dataRoot);
        return success ? 0 : 1;
    }

    // Interactive GUI mode
    runApp(nullptr, ysContextObject::DeviceAPI::OpenGL4_0);
    return 0;
}
