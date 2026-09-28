#include <gtest/gtest.h>
#include "../include/export_recipe.h"

TEST(ExportRecipeTests, ParseGameVehiclesRecipe) {
    ExportRecipe recipe;
    std::string err;
    bool success = ExportRecipe::loadFromFile("recipes/game_vehicles.json", recipe, err);
    EXPECT_TRUE(success) << "Failed to load recipes/game_vehicles.json: " << err;

    EXPECT_EQ(recipe.globalSettings.sampleRate, 44100);
    EXPECT_EQ(recipe.globalSettings.outputDir, "assets/audio/engines");
    EXPECT_TRUE(recipe.globalSettings.embedLoopMarkers);

    EXPECT_EQ(recipe.vehicles.size(), 21);
    EXPECT_EQ(recipe.vehicles[0].id, "trench");
    EXPECT_EQ(recipe.vehicles[1].id, "aileron");
    EXPECT_EQ(recipe.vehicles[1].exportProfile.rpmMin, 900);
    EXPECT_EQ(recipe.vehicles[1].exportProfile.rpmMax, 6800);
    EXPECT_EQ(recipe.vehicles[1].exportProfile.rpmStep, 500);
    EXPECT_TRUE(recipe.vehicles[1].exportProfile.exportRevBlip);
    EXPECT_TRUE(recipe.vehicles[1].exportProfile.exportRevLimiter);
    EXPECT_EQ(recipe.vehicles[17].id, "mamba");
    EXPECT_EQ(recipe.vehicles[18].id, "phantom");
    EXPECT_EQ(recipe.vehicles[19].id, "scorpio");
    EXPECT_EQ(recipe.vehicles[20].id, "pulse");
}

TEST(ExportRecipeTests, CreateDefaultSingleEngine) {
    ExportRecipe recipe = ExportRecipe::createDefaultSingleEngine("assets/engines/chevrolet/chev_truck_454.mr", "out/test", 250);
    EXPECT_EQ(recipe.vehicles.size(), 1);
    EXPECT_EQ(recipe.vehicles[0].id, "chev_truck_454");
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmStep, 250);
}

TEST(ExportRecipeTests, ApplySfxFilter) {
    ExportRecipe recipe;
    std::string err;
    bool success = ExportRecipe::loadFromFile("recipes/game_vehicles.json", recipe, err);
    EXPECT_TRUE(success);

    // Apply decel_crackle filter
    EXPECT_TRUE(recipe.applySfxFilter("decel_crackle", err));
    for (const auto &v : recipe.vehicles) {
        EXPECT_FALSE(v.exportProfile.exportSteadyRpm);
        EXPECT_FALSE(v.exportProfile.exportStarter);
        EXPECT_FALSE(v.exportProfile.exportRevLimiter);
        EXPECT_FALSE(v.exportProfile.exportRevBlip);
        EXPECT_TRUE(v.exportProfile.exportDecelCrackle);
    }

    // Apply rev_blip filter
    EXPECT_TRUE(recipe.applySfxFilter("rev_blip", err));
    for (const auto &v : recipe.vehicles) {
        EXPECT_FALSE(v.exportProfile.exportSteadyRpm);
        EXPECT_FALSE(v.exportProfile.exportStarter);
        EXPECT_FALSE(v.exportProfile.exportRevLimiter);
        EXPECT_TRUE(v.exportProfile.exportRevBlip);
        EXPECT_FALSE(v.exportProfile.exportDecelCrackle);
    }

    // Filter by vehicle
    EXPECT_TRUE(recipe.filterVehicles("trench"));
    EXPECT_EQ(recipe.vehicles.size(), 1);
    EXPECT_EQ(recipe.vehicles[0].id, "trench");

    // Invalid SFX filter
    EXPECT_FALSE(recipe.applySfxFilter("non_existent_sfx", err));
}

