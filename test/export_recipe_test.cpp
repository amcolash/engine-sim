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

    EXPECT_EQ(recipe.vehicles.size(), 10);
    EXPECT_EQ(recipe.vehicles[0].id, "aileron");
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmMin, 900);
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmMax, 6800);
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmStep, 500);
    EXPECT_TRUE(recipe.vehicles[0].exportProfile.exportRevBlip);
    EXPECT_TRUE(recipe.vehicles[0].exportProfile.exportRevLimiter);
    EXPECT_EQ(recipe.vehicles[6].id, "mantis");
    EXPECT_EQ(recipe.vehicles[7].id, "pulse");
    EXPECT_EQ(recipe.vehicles[8].id, "glacier");
    EXPECT_EQ(recipe.vehicles[9].id, "chariot");
}

TEST(ExportRecipeTests, CreateDefaultSingleEngine) {
    ExportRecipe recipe = ExportRecipe::createDefaultSingleEngine("assets/engines/chevrolet/chev_truck_454.mr", "out/test", 250);
    EXPECT_EQ(recipe.vehicles.size(), 1);
    EXPECT_EQ(recipe.vehicles[0].id, "chev_truck_454");
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmStep, 250);
}

