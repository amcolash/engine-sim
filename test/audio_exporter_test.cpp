#include <gtest/gtest.h>
#include "../include/audio_exporter.h"
#include "../include/export_recipe.h"

#include <filesystem>
#include <fstream>

TEST(AudioExporterTests, ExportMiniProfile) {
    const std::string testOutputDir = "test_export_output";
    std::filesystem::remove_all(testOutputDir);

    ExportRecipe recipe;
    recipe.globalSettings.outputDir = testOutputDir;
    recipe.globalSettings.sampleRate = 44100;
    recipe.globalSettings.embedLoopMarkers = true;
    recipe.globalSettings.normalizePeakDbfs = -1.0f;
    recipe.defaultSearchPaths = { "assets", "part-library" };

    VehicleExportConfig v;
    v.id = "test_i4";
    v.displayName = "Test Inline-4";
    v.scriptPath = "assets/engines/atg-video-1/05_honda_vtec.mr";
    v.exportProfile.exportSteadyRpm = true;
    v.exportProfile.rpmMin = 1000;
    v.exportProfile.rpmMax = 2000;
    v.exportProfile.rpmStep = 1000;
    v.exportProfile.cyclesPerLoop = 4;
    v.exportProfile.exportStarter = true;
    v.exportProfile.exportRevLimiter = false;
    v.exportProfile.exportRevBlip = true;
    v.exportProfile.exportDecelCrackle = false;

    recipe.vehicles.push_back(v);

    AudioExporter exporter;
    bool success = exporter.exportRecipe(recipe, ".");
    EXPECT_TRUE(success);

    // Check generated files
    const std::string vehicleDir = testOutputDir + "/test_i4";
    EXPECT_TRUE(std::filesystem::exists(vehicleDir + "/rpm_1000.wav"));
    EXPECT_TRUE(std::filesystem::exists(vehicleDir + "/rpm_2000.wav"));
    EXPECT_TRUE(std::filesystem::exists(vehicleDir + "/starter_crank.wav"));
    EXPECT_TRUE(std::filesystem::exists(vehicleDir + "/engine_start.wav"));
    EXPECT_TRUE(std::filesystem::exists(vehicleDir + "/rev_blip.wav"));
    EXPECT_TRUE(std::filesystem::exists(vehicleDir + "/manifest.json"));

    // Check manifest JSON content
    std::ifstream mf(vehicleDir + "/manifest.json");
    EXPECT_TRUE(mf.is_open());
    std::string manifestContent((std::istreambuf_iterator<char>(mf)), std::istreambuf_iterator<char>());
    EXPECT_NE(manifestContent.find("test_i4"), std::string::npos);
    EXPECT_NE(manifestContent.find("rpm_1000.wav"), std::string::npos);

    std::filesystem::remove_all(testOutputDir);
}

