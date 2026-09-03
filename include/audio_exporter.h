#ifndef ATG_ENGINE_SIM_AUDIO_EXPORTER_H
#define ATG_ENGINE_SIM_AUDIO_EXPORTER_H

#include "export_recipe.h"
#include "piston_engine_simulator.h"
#include "wav_writer.h"

#include <string>
#include <vector>
#include <functional>

class AudioExporter {
public:
    struct ProgressCallback {
        std::function<void(const std::string &vehicleId, const std::string &stage, float progress)> onProgress;
    };

    AudioExporter();
    ~AudioExporter();

    bool exportRecipe(
        const ExportRecipe &recipe,
        const std::string &dataRoot,
        ProgressCallback *callback = nullptr);

    bool exportVehicle(
        const VehicleExportConfig &vehicleConfig,
        const GlobalExportSettings &globalSettings,
        const std::vector<std::string> &searchPaths,
        const std::string &dataRoot,
        ProgressCallback *callback = nullptr);

private:
    struct RenderedLoop {
        int targetRpm = 0;
        std::string fileName;
        std::vector<float> samples;
        double durationSec = 0.0;
        uint32_t loopStartSample = 0;
        uint32_t loopEndSample = 0;
    };

    struct RenderedTransient {
        std::string type;
        std::string fileName;
        std::vector<float> samples;
        double durationSec = 0.0;
    };

    bool initializeAndStartEngine(
        PistonEngineSimulator &sim,
        Engine *engine);

    void ensureEngineRunning(
        PistonEngineSimulator &sim);

    void drainAudio(
        PistonEngineSimulator &sim,
        std::vector<float> *recordedSamples = nullptr);

    bool runSimulationSteps(
        PistonEngineSimulator &sim,
        double durationSec,
        std::vector<float> *recordedSamples = nullptr);

    bool captureSteadyRpmLoop(
        PistonEngineSimulator &sim,
        int targetRpm,
        int cyclesToCapture,
        std::vector<float> &outSamples);

    bool generateStarterCrank(
        PistonEngineSimulator &sim,
        double durationSec,
        std::vector<float> &outSamples);

    bool generateEngineStart(
        PistonEngineSimulator &sim,
        std::vector<float> &outSamples);

    bool generateRevLimiter(
        PistonEngineSimulator &sim,
        double durationSec,
        std::vector<float> &outSamples);

    bool generateThrottleBlip(
        PistonEngineSimulator &sim,
        std::vector<float> &outSamples);

    bool generateDecelCrackle(
        PistonEngineSimulator &sim,
        int initialRpm,
        double durationSec,
        std::vector<float> &outSamples);

    void writeManifest(
        const std::string &manifestPath,
        const VehicleExportConfig &vehicleConfig,
        const GlobalExportSettings &globalSettings,
        const std::vector<RenderedLoop> &loops,
        const std::vector<RenderedTransient> &transients);
};

#endif /* ATG_ENGINE_SIM_AUDIO_EXPORTER_H */

