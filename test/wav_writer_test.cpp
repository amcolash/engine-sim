#include <gtest/gtest.h>
#include "../include/wav_writer.h"

#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>

TEST(WavWriterTests, NormalizationTest) {
    std::vector<float> samples = { 0.1f, -0.5f, 0.2f, -0.2f };
    WavWriter::normalizeAudio(samples, 0.0f); // 0.0 dBFS = 1.0 peak
    float maxVal = 0.0f;
    for (float s : samples) maxVal = std::max(maxVal, std::abs(s));
    EXPECT_NEAR(maxVal, 1.0f, 1e-4);
}

TEST(WavWriterTests, MicroCrossfadeTest) {
    std::vector<float> samples(200, 1.0f);
    samples[0] = 0.0f; // Discontinuity
    samples[samples.size() - 16] = 0.5f;
    WavWriter::applyMicroCrossfade(samples, 16);
    EXPECT_NEAR(samples[0], 0.5f, 1e-4);
}

TEST(WavWriterTests, WriteWavWithSmplLoop) {
    const std::string testPath = "test_output_loop.wav";
    std::vector<float> samples(44100);
    for (size_t i = 0; i < samples.size(); ++i) {
        samples[i] = std::sin(2.0 * 3.1415926535 * 440.0 * i / 44100.0);
    }

    WavWriter::WriteOptions options;
    options.sampleRate = 44100;
    options.bitsPerSample = 16;
    options.normalize = true;
    options.targetPeakDbfs = -1.0f;
    options.loop.enabled = true;
    options.loop.startSample = 0;
    options.loop.endSample = 44099;

    bool written = WavWriter::writeWav(testPath, samples, options);
    EXPECT_TRUE(written);

    std::ifstream file(testPath, std::ios::binary);
    EXPECT_TRUE(file.is_open());

    char riffHeader[4];
    file.read(riffHeader, 4);
    EXPECT_EQ(std::string(riffHeader, 4), "RIFF");

    file.close();
    std::filesystem::remove(testPath);
}
