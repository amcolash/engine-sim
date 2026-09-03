#include "../include/wav_writer.h"

#include <fstream>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <filesystem>

namespace {

void writeUint32LE(std::ofstream &stream, uint32_t value) {
    char bytes[4];
    bytes[0] = static_cast<char>(value & 0xFF);
    bytes[1] = static_cast<char>((value >> 8) & 0xFF);
    bytes[2] = static_cast<char>((value >> 16) & 0xFF);
    bytes[3] = static_cast<char>((value >> 24) & 0xFF);
    stream.write(bytes, 4);
}

void writeUint16LE(std::ofstream &stream, uint16_t value) {
    char bytes[2];
    bytes[0] = static_cast<char>(value & 0xFF);
    bytes[1] = static_cast<char>((value >> 8) & 0xFF);
    stream.write(bytes, 2);
}

} // anonymous namespace

void WavWriter::normalizeAudio(std::vector<float> &audioSamples, float targetPeakDbfs) {
    if (audioSamples.empty()) return;

    float maxVal = 0.0f;
    for (float sample : audioSamples) {
        maxVal = std::max(maxVal, std::abs(sample));
    }

    if (maxVal <= 1e-6f) return;

    const float targetPeak = std::pow(10.0f, targetPeakDbfs / 20.0f);
    const float gain = targetPeak / maxVal;

    for (float &sample : audioSamples) {
        sample *= gain;
    }
}

void WavWriter::applyMicroCrossfade(std::vector<float> &audioSamples, size_t crossfadeSamples) {
    const size_t totalSamples = audioSamples.size();
    if (totalSamples < crossfadeSamples * 2 || crossfadeSamples == 0) return;

    for (size_t i = 0; i < crossfadeSamples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(crossfadeSamples);
        const float headVal = audioSamples[i];
        const float tailVal = audioSamples[totalSamples - crossfadeSamples + i];

        audioSamples[i] = static_cast<float>((1.0 - t) * tailVal + t * headVal);
    }
}

bool WavWriter::writeWav(
    const std::string &filePath,
    const std::vector<float> &audioSamples,
    const WriteOptions &options)
{
    if (audioSamples.empty()) return false;

    // Create parent directories if needed
    try {
        std::filesystem::path path(filePath);
        if (path.has_parent_path()) {
            std::filesystem::create_directories(path.parent_path());
        }
    } catch (...) {
        // Ignore and attempt open
    }

    std::vector<float> processedSamples = audioSamples;
    if (options.normalize) {
        normalizeAudio(processedSamples, options.targetPeakDbfs);
    }

    std::ofstream stream(filePath, std::ios::binary);
    if (!stream.is_open()) return false;

    const uint32_t numSamples = static_cast<uint32_t>(processedSamples.size());
    const uint16_t numChannels = static_cast<uint16_t>(options.channels);
    const uint16_t bitsPerSample = static_cast<uint16_t>(options.bitsPerSample);
    const uint32_t sampleRate = static_cast<uint32_t>(options.sampleRate);
    const uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
    const uint16_t blockAlign = numChannels * (bitsPerSample / 8);
    const uint32_t dataChunkSize = numSamples * numChannels * (bitsPerSample / 8);

    const bool hasSmpl = options.loop.enabled;
    const uint32_t smplChunkSize = hasSmpl ? (36 + 24) : 0; // 36 header + 1 loop struct of 24 bytes
    const uint32_t totalFileSize = 4 + (8 + 16) + (hasSmpl ? (8 + smplChunkSize) : 0) + (8 + dataChunkSize);

    // 1. RIFF Header
    stream.write("RIFF", 4);
    writeUint32LE(stream, totalFileSize);
    stream.write("WAVE", 4);

    // 2. fmt chunk
    stream.write("fmt ", 4);
    writeUint32LE(stream, 16); // PCM chunk size
    writeUint16LE(stream, 1);  // PCM format
    writeUint16LE(stream, numChannels);
    writeUint32LE(stream, sampleRate);
    writeUint32LE(stream, byteRate);
    writeUint16LE(stream, blockAlign);
    writeUint16LE(stream, bitsPerSample);

    // 3. smpl chunk (if loop enabled)
    if (hasSmpl) {
        stream.write("smpl", 4);
        writeUint32LE(stream, smplChunkSize);
        writeUint32LE(stream, 0); // Manufacturer
        writeUint32LE(stream, 0); // Product
        writeUint32LE(stream, 1000000000U / sampleRate); // Sample period in nanoseconds
        writeUint32LE(stream, 60); // MIDI unity note (Middle C)
        writeUint32LE(stream, 0);  // MIDI pitch fraction
        writeUint32LE(stream, 0);  // SMPTE format
        writeUint32LE(stream, 0);  // SMPTE offset
        writeUint32LE(stream, 1);  // Num sample loops
        writeUint32LE(stream, 0);  // Sampler data size

        // Loop record
        writeUint32LE(stream, 0); // Cue point ID
        writeUint32LE(stream, 0); // Type: 0 = normal forward loop
        writeUint32LE(stream, options.loop.startSample);
        writeUint32LE(stream, options.loop.endSample > 0 ? options.loop.endSample : (numSamples - 1));
        writeUint32LE(stream, 0); // Fraction
        writeUint32LE(stream, 0); // Play count: 0 = infinite loop
    }

    // 4. data chunk
    stream.write("data", 4);
    writeUint32LE(stream, dataChunkSize);

    // Write samples
    if (bitsPerSample == 16) {
        for (float sample : processedSamples) {
            float clamped = std::clamp(sample, -1.0f, 1.0f);
            int16_t pcm16 = static_cast<int16_t>(std::lround(clamped * 32767.0f));
            writeUint16LE(stream, static_cast<uint16_t>(pcm16));
        }
    } else if (bitsPerSample == 24) {
        for (float sample : processedSamples) {
            float clamped = std::clamp(sample, -1.0f, 1.0f);
            int32_t pcm24 = static_cast<int32_t>(std::lround(clamped * 8388607.0f));
            char bytes[3];
            bytes[0] = static_cast<char>(pcm24 & 0xFF);
            bytes[1] = static_cast<char>((pcm24 >> 8) & 0xFF);
            bytes[2] = static_cast<char>((pcm24 >> 16) & 0xFF);
            stream.write(bytes, 3);
        }
    }

    return stream.good();
}

bool WavWriter::writeWav(
    const std::string &filePath,
    const std::vector<int16_t> &audioSamples,
    const WriteOptions &options)
{
    std::vector<float> floatSamples(audioSamples.size());
    for (size_t i = 0; i < audioSamples.size(); ++i) {
        floatSamples[i] = static_cast<float>(audioSamples[i]) / 32768.0f;
    }
    return writeWav(filePath, floatSamples, options);
}
