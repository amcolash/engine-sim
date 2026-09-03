#include "../include/export_recipe.h"
#include "../dependencies/libraries/json/nlohmann/json.hpp"

#include <fstream>
#include <sstream>
#include <filesystem>

using json = nlohmann::json;

bool ExportRecipe::loadFromFile(const std::string &filePath, ExportRecipe &recipe, std::string &errorMessage) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        errorMessage = "Could not open recipe file: " + filePath;
        return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return parseJsonString(buffer.str(), recipe, errorMessage);
}

bool ExportRecipe::parseJsonString(const std::string &jsonStr, ExportRecipe &recipe, std::string &errorMessage) {
    try {
        json j = json::parse(jsonStr);

        // Global settings
        if (j.contains("global_settings")) {
            auto &gs = j["global_settings"];
            if (gs.contains("output_dir")) recipe.globalSettings.outputDir = gs["output_dir"].get<std::string>();
            if (gs.contains("sample_rate")) recipe.globalSettings.sampleRate = gs["sample_rate"].get<int>();
            if (gs.contains("bits_per_sample")) recipe.globalSettings.bitsPerSample = gs["bits_per_sample"].get<int>();
            if (gs.contains("normalize_peak_dbfs")) recipe.globalSettings.normalizePeakDbfs = gs["normalize_peak_dbfs"].get<float>();
            if (gs.contains("embed_loop_markers")) recipe.globalSettings.embedLoopMarkers = gs["embed_loop_markers"].get<bool>();
            if (gs.contains("default_cycles_per_loop")) recipe.globalSettings.defaultCyclesPerLoop = gs["default_cycles_per_loop"].get<int>();
            if (gs.contains("default_min_loop_duration_sec")) recipe.globalSettings.defaultMinLoopDurationSec = gs["default_min_loop_duration_sec"].get<double>();
            if (gs.contains("min_loop_duration_sec")) recipe.globalSettings.defaultMinLoopDurationSec = gs["min_loop_duration_sec"].get<double>();
        }

        // Global search paths
        if (j.contains("search_paths") && j["search_paths"].is_array()) {
            recipe.defaultSearchPaths.clear();
            for (const auto &p : j["search_paths"]) {
                recipe.defaultSearchPaths.push_back(p.get<std::string>());
            }
        }

        // Vehicles
        if (j.contains("vehicles") && j["vehicles"].is_array()) {
            recipe.vehicles.clear();
            for (const auto &v : j["vehicles"]) {
                VehicleExportConfig vc;
                if (v.contains("id")) vc.id = v["id"].get<std::string>();
                if (v.contains("display_name")) vc.displayName = v["display_name"].get<std::string>();
                if (v.contains("script_path")) vc.scriptPath = v["script_path"].get<std::string>();

                if (v.contains("search_paths") && v["search_paths"].is_array()) {
                    for (const auto &p : v["search_paths"]) {
                        vc.extraSearchPaths.push_back(p.get<std::string>());
                    }
                }

                vc.exportProfile.cyclesPerLoop = recipe.globalSettings.defaultCyclesPerLoop;
                vc.exportProfile.minLoopDurationSec = recipe.globalSettings.defaultMinLoopDurationSec;

                if (v.contains("export_profile")) {
                    auto &ep = v["export_profile"];
                    if (ep.contains("rpm_min")) vc.exportProfile.rpmMin = ep["rpm_min"].get<int>();
                    if (ep.contains("rpm_max")) vc.exportProfile.rpmMax = ep["rpm_max"].get<int>();
                    if (ep.contains("rpm_step")) vc.exportProfile.rpmStep = ep["rpm_step"].get<int>();
                    if (ep.contains("cycles_per_loop")) vc.exportProfile.cyclesPerLoop = ep["cycles_per_loop"].get<int>();
                    if (ep.contains("min_loop_duration_sec")) vc.exportProfile.minLoopDurationSec = ep["min_loop_duration_sec"].get<double>();
                    if (ep.contains("export_steady_rpm")) vc.exportProfile.exportSteadyRpm = ep["export_steady_rpm"].get<bool>();
                    if (ep.contains("export_starter")) vc.exportProfile.exportStarter = ep["export_starter"].get<bool>();
                    if (ep.contains("export_rev_limiter")) vc.exportProfile.exportRevLimiter = ep["export_rev_limiter"].get<bool>();
                    if (ep.contains("export_rev_blip")) vc.exportProfile.exportRevBlip = ep["export_rev_blip"].get<bool>();
                    if (ep.contains("export_decel_crackle")) vc.exportProfile.exportDecelCrackle = ep["export_decel_crackle"].get<bool>();

                    if (ep.contains("rpms") && ep["rpms"].is_array()) {
                        for (const auto &rpmVal : ep["rpms"]) {
                            vc.exportProfile.explicitRpms.push_back(rpmVal.get<int>());
                        }
                    }
                }

                recipe.vehicles.push_back(vc);
            }
        }

        return true;
    } catch (const std::exception &e) {
        errorMessage = std::string("JSON parsing error: ") + e.what();
        return false;
    }
}

ExportRecipe ExportRecipe::createDefaultSingleEngine(const std::string &scriptPath, const std::string &outputDir, int rpmStep) {
    ExportRecipe recipe;
    recipe.globalSettings.outputDir = outputDir;

    std::filesystem::path p(scriptPath);
    std::string stem = p.stem().string();

    VehicleExportConfig vc;
    vc.id = stem;
    vc.displayName = stem;
    vc.scriptPath = scriptPath;
    vc.exportProfile.rpmMin = 1000;
    vc.exportProfile.rpmMax = 7000;
    vc.exportProfile.rpmStep = rpmStep > 0 ? rpmStep : 500;
    vc.exportProfile.cyclesPerLoop = 6;

    recipe.vehicles.push_back(vc);
    return recipe;
}

