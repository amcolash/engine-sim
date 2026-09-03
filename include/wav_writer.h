#ifndef ATG_ENGINE_SIM_WAV_WRITER_H
#define ATG_ENGINE_SIM_WAV_WRITER_H

#include <string>
#include <vector>
#include <cstdint>

class WavWriter {
public:
    struct LoopPoints {
        bool enabled = false;
        uint32_t startSample = 0;
        uint32_t endSample = 0;
    };

    struct WriteOptions {
        int sampleRate = 44100;
        int bitsPerSample = 16;
        int channels = 1;
        bool normalize = true;
        float targetPeakDbfs = -1.0f; // -1.0 dBFS standard ceiling
        LoopPoints loop;
    };

    static bool writeWav(
        const std::string &filePath,
        const std::vector<float> &audioSamples,
        const WriteOptions &options);

    static bool writeWav(
        const std::string &filePath,
        const std::vector<int16_t> &audioSamples,
        const WriteOptions &options);

    static void normalizeAudio(
        std::vector<float> &audioSamples,
        float targetPeakDbfs);

    static void applyMicroCrossfade(
        std::vector<float> &audioSamples,
        size_t crossfadeSamples = 32);
};

#endif /* ATG_ENGINE_SIM_WAV_WRITER_H */
