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

    EXPECT_GE(recipe.vehicles.size(), 6);
    EXPECT_EQ(recipe.vehicles[0].id, "grizzly");
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmMin, 1000);
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmMax, 5000);
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmStep, 500);
    EXPECT_TRUE(recipe.vehicles[0].exportProfile.exportStarter);
    EXPECT_TRUE(recipe.vehicles[0].exportProfile.exportRevLimiter);
}

TEST(ExportRecipeTests, CreateDefaultSingleEngine) {
    ExportRecipe recipe = ExportRecipe::createDefaultSingleEngine("assets/engines/chevrolet/chev_truck_454.mr", "out/test", 250);
    EXPECT_EQ(recipe.vehicles.size(), 1);
    EXPECT_EQ(recipe.vehicles[0].id, "chev_truck_454");
    EXPECT_EQ(recipe.vehicles[0].exportProfile.rpmStep, 250);
}

