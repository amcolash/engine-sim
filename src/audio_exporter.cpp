#include "../include/audio_exporter.h"
#include "../dependencies/libraries/json/nlohmann/json.hpp"
#include "compiler.h"
#include "piranha.h"
#include "yds_audio_wave_file.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <thread>
#include <chrono>
#include <regex>

using json = nlohmann::json;

AudioExporter::AudioExporter() {
    /* void */
}

AudioExporter::~AudioExporter() {
    /* void */
}

void AudioExporter::drainAudio(PistonEngineSimulator &sim, std::vector<float> *recordedSamples) {
    int16_t buffer[2048];
    while (true) {
        int read = sim.readAudioOutput(2048, buffer);
        if (read <= 0) break;
        if (recordedSamples != nullptr) {
            for (int i = 0; i < read; ++i) {
                recordedSamples->push_back(static_cast<float>(buffer[i]) / 32768.0f);
            }
        }
        if (read < 2048) break;
    }
}

bool AudioExporter::runSimulationSteps(
    PistonEngineSimulator &sim,
    double durationSec,
    std::vector<float> *recordedSamples)
{
    const double frameDt = 0.02; // 20ms frame
    for (double t = 0.0; t < durationSec; t += frameDt) {
        sim.startFrame(frameDt);
        while (sim.simulateStep()) {
            /* physics step */
        }
        sim.endFrame();
        drainAudio(sim, recordedSamples);
    }

    // Final drain
    for (int i = 0; i < 5; ++i) {
        drainAudio(sim, recordedSamples);
    }

    return true;
}

bool AudioExporter::initializeAndStartEngine(PistonEngineSimulator &sim, Engine *engine) {
    if (!engine) return false;

    // Start with starter motor engaged and ignition module enabled
    engine->getIgnitionModule()->m_enabled = true;
    sim.m_starterMotor.m_enabled = true;
    sim.m_starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
    sim.m_starterMotor.m_maxTorque = engine->getStarterTorque();
    engine->setSpeedControl(0.35);

    // Crank until engine starts and speed exceeds 600 RPM
    const double frameDt = 0.02;
    for (double t = 0.0; t < 2.0; t += frameDt) {
        sim.startFrame(frameDt);
        while (sim.simulateStep()) {
            if (engine->getSpeed() > units::rpm(600)) {
                sim.m_starterMotor.m_enabled = false;
            }
        }
        sim.endFrame();
        drainAudio(sim, nullptr);
    }
    sim.m_starterMotor.m_enabled = false;

    // Settle at idle for 1.0 second
    engine->setSpeedControl(0.0);
    runSimulationSteps(sim, 1.0, nullptr);

    return true;
}

bool AudioExporter::captureSteadyRpmLoop(
    PistonEngineSimulator &sim,
    int targetRpm,
    int cyclesToCapture,
    double minDurationSec,
    std::vector<float> &outSamples)
{
    Engine *engine = sim.getEngine();
    if (!engine) return false;

    outSamples.clear();

    // In a 4-stroke engine, 1 combustion cycle (720 deg) = 120 / RPM seconds.
    // Ensure we capture enough complete cycles to meet or exceed minDurationSec (e.g. >= 1.0s).
    int totalCycles = cyclesToCapture;
    if (minDurationSec > 0.0 && targetRpm > 0) {
        double secondsPerCycle = 120.0 / static_cast<double>(targetRpm);
        int requiredCycles = static_cast<int>(std::ceil(minDurationSec / secondsPerCycle));
        totalCycles = std::max(cyclesToCapture, requiredCycles);
    }

    // Configure virtual dynamometer to hold exact target RPM
    sim.m_dyno.m_enabled = true;
    sim.m_dyno.m_hold = true;
    sim.m_dyno.m_rotationSpeed = units::rpm(targetRpm);
    sim.m_dyno.m_maxTorque = 20000.0;
    sim.m_dyno.m_ks = 2000.0;
    sim.m_dyno.m_kd = 20.0;

    // Set sufficient throttle to keep cylinders firing against dyno load
    double throttle = std::clamp(0.25 + 0.60 * (static_cast<double>(targetRpm) / 8000.0), 0.25, 0.90);
    engine->setSpeedControl(throttle);

    // Warm up & let dyno lock speed (1.0s)
    runSimulationSteps(sim, 1.0, nullptr);
    drainAudio(sim, nullptr); // Discard warmup audio

    // Track crankshaft cycle angle (0 to 4*pi for 4-stroke 720 deg)
    Crankshaft *crank = engine->getOutputCrankshaft();
    const double wrapThreshold = 2.0 * constants::pi;

    // 1. Wait for angle to wrap around 0 / 4*pi boundary
    double lastAngle = crank->getCycleAngle();
    bool zeroCrossed = false;
    const double frameDt = 0.005; // 5ms high-precision frame
    for (int frame = 0; frame < 500; ++frame) {
        sim.startFrame(frameDt);
        while (sim.simulateStep()) {
            double angle = crank->getCycleAngle();
            if (std::abs(angle - lastAngle) > wrapThreshold) {
                zeroCrossed = true;
                break;
            }
            lastAngle = angle;
        }
        sim.endFrame();
        drainAudio(sim, nullptr);
        if (zeroCrossed) break;
    }

    if (!zeroCrossed) return false;

    // 2. Record exactly N full 720-degree combustion cycles
    int completedCycles = 0;
    lastAngle = crank->getCycleAngle();

    for (int frame = 0; frame < 4000; ++frame) {
        sim.startFrame(frameDt);
        bool finished = false;
        while (sim.simulateStep()) {
            double angle = crank->getCycleAngle();

            // Check cycle boundary crossing
            if (std::abs(angle - lastAngle) > wrapThreshold) {
                completedCycles++;
                if (completedCycles >= totalCycles) {
                    finished = true;
                    break;
                }
            }
            lastAngle = angle;
        }
        sim.endFrame();
        drainAudio(sim, &outSamples);

        if (finished) break;
    }

    // Drain any remaining tail
    for (int i = 0; i < 5; ++i) {
        drainAudio(sim, &outSamples);
    }

    // Apply micro-crossfade at the seam
    WavWriter::applyMicroCrossfade(outSamples, 32);

    return !outSamples.empty();
}

void AudioExporter::ensureEngineRunning(PistonEngineSimulator &sim) {
    Engine *engine = sim.getEngine();
    if (!engine) return;

    // Disengage transmission
    if (sim.getTransmission()) {
        sim.getTransmission()->changeGear(-1);
        sim.getTransmission()->setClutchPressure(0.0);
    }

    // Completely disable dyno — free-running engine
    sim.m_dyno.m_enabled = false;
    sim.m_dyno.m_hold = false;

    // Enable ignition
    engine->getIgnitionModule()->m_enabled = true;

    // If engine is not running or running too slowly, crank it forward
    if (engine->getSpeed() < units::rpm(600)) {
        sim.m_starterMotor.m_enabled = true;
        sim.m_starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
        sim.m_starterMotor.m_maxTorque = engine->getStarterTorque();
        engine->setSpeedControl(0.25);

        const double frameDt = 0.02;
        for (double t = 0.0; t < 0.8; t += frameDt) {
            sim.startFrame(frameDt);
            while (sim.simulateStep()) {}
            sim.endFrame();
            drainAudio(sim, nullptr);
        }
        sim.m_starterMotor.m_enabled = false;
    }

    // Drop to natural idle (0.0) and allow 0.8s to settle naturally
    engine->setSpeedControl(0.0);
    const double frameDt = 0.02;
    for (double t = 0.0; t < 0.8; t += frameDt) {
        sim.startFrame(frameDt);
        while (sim.simulateStep()) {}
        sim.endFrame();
        drainAudio(sim, nullptr);
    }

    // Safety fallback: if engine RPM fell below 500, crank once more
    if (engine->getSpeed() < units::rpm(500)) {
        sim.m_starterMotor.m_enabled = true;
        sim.m_starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
        sim.m_starterMotor.m_maxTorque = engine->getStarterTorque();
        engine->setSpeedControl(0.30);
        for (double t = 0.0; t < 0.8; t += frameDt) {
            sim.startFrame(frameDt);
            while (sim.simulateStep()) {}
            sim.endFrame();
            drainAudio(sim, nullptr);
        }
        sim.m_starterMotor.m_enabled = false;
        engine->setSpeedControl(0.0);
        for (double t = 0.0; t < 0.5; t += frameDt) {
            sim.startFrame(frameDt);
            while (sim.simulateStep()) {}
            sim.endFrame();
            drainAudio(sim, nullptr);
        }
    }
}


bool AudioExporter::generateEngineStart(
    PistonEngineSimulator &sim,
    std::vector<float> &outSamples)
{
    Engine *engine = sim.getEngine();
    if (!engine) return false;

    outSamples.clear();

    // 1. Ensure dyno & transmission are completely disengaged
    sim.m_dyno.m_enabled = false;
    sim.m_dyno.m_hold = false;
    if (sim.getTransmission()) {
        sim.getTransmission()->changeGear(-1);
        sim.getTransmission()->setClutchPressure(0.0);
    }

    // Turn off ignition and let engine come to a complete standstill at 0 RPM
    engine->getIgnitionModule()->m_enabled = false;
    engine->setSpeedControl(0.0);
    runSimulationSteps(sim, 0.8, nullptr);
    drainAudio(sim, nullptr);

    // Let any synthesizer convolution impulse response ringout settle into silence
    for (int frame = 0; frame < 20; ++frame) {
        sim.startFrame(0.02);
        while (sim.simulateStep()) {}
        sim.endFrame();
        drainAudio(sim, nullptr);
    }

    // 2. Turn on ignition and engage starter motor (forward direction)
    engine->getIgnitionModule()->m_enabled = true;
    sim.m_starterMotor.m_enabled = true;
    sim.m_starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
    sim.m_starterMotor.m_maxTorque = engine->getStarterTorque();
    engine->setSpeedControl(0.20);

    // 3. Crank until engine catches and stabilizes above starter speed
    const double frameDt = 0.01;
    for (double t = 0.0; t < 1.5; t += frameDt) {
        sim.startFrame(frameDt);
        while (sim.simulateStep()) {
            if (engine->getSpeed() > units::rpm(750)) {
                sim.m_starterMotor.m_enabled = false;
            }
        }
        sim.endFrame();
        drainAudio(sim, &outSamples);
        if (!sim.m_starterMotor.m_enabled && t > 0.4) break;
    }
    sim.m_starterMotor.m_enabled = false;

    // 4. Settle at natural idle for 0.5s (no dyno)
    engine->setSpeedControl(0.0);
    runSimulationSteps(sim, 0.50, &outSamples);

    // 5. 100ms fade-in, 200ms fade-out
    const size_t fadeInSamples = static_cast<size_t>(0.10 * 44100);
    const size_t fadeOutSamples = static_cast<size_t>(0.20 * 44100);
    WavWriter::applyEnvelopeFade(outSamples, fadeInSamples, fadeOutSamples);

    return !outSamples.empty();
}

bool AudioExporter::generateRevLimiter(
    PistonEngineSimulator &sim,
    double durationSec,
    std::vector<float> &outSamples)
{
    Engine *engine = sim.getEngine();
    if (!engine) return false;

    outSamples.clear();

    // Ensure engine is running at idle
    ensureEngineRunning(sim);

    const double redlineRpm = units::toRpm(engine->getRedline());
    const double bounceLowRpm = std::max(1500.0, redlineRpm - 500.0);

    // Warm up to redline (0.5s)
    sim.m_dyno.m_enabled = true;
    sim.m_dyno.m_hold = true;
    sim.m_dyno.m_rotationSpeed = engine->getRedline();
    sim.m_dyno.m_maxTorque = 8000.0;
    engine->getIgnitionModule()->m_enabled = true;
    engine->setSpeedControl(1.0);

    runSimulationSteps(sim, 0.5, nullptr);
    drainAudio(sim, nullptr);

    // Record rapid bouncing between redline and redline - 500 RPM
    const double bouncePeriod = 0.12; // 120ms bounce cycle (~8.3 Hz)
    const double halfPeriod = bouncePeriod * 0.5;
    const double frameDt = 0.01;

    double elapsed = 0.0;
    while (elapsed < durationSec) {
        // High surge towards redline with WOT
        sim.m_dyno.m_rotationSpeed = units::rpm(redlineRpm);
        engine->setSpeedControl(1.0);
        for (double t = 0.0; t < halfPeriod && elapsed < durationSec; t += frameDt, elapsed += frameDt) {
            sim.startFrame(frameDt);
            while (sim.simulateStep()) {}
            sim.endFrame();
            drainAudio(sim, &outSamples);
        }

        // Ignition cut / overrun dip down 500 RPM
        sim.m_dyno.m_rotationSpeed = units::rpm(bounceLowRpm);
        engine->setSpeedControl(0.10);
        for (double t = 0.0; t < halfPeriod && elapsed < durationSec; t += frameDt, elapsed += frameDt) {
            sim.startFrame(frameDt);
            while (sim.simulateStep()) {}
            sim.endFrame();
            drainAudio(sim, &outSamples);
        }
    }

    // Apply seamless micro-crossfade and smooth boundary envelopes
    WavWriter::applyMicroCrossfade(outSamples, 64);
    WavWriter::applyEnvelopeFade(outSamples, 256, 1024);

    // Return throttle to idle and settle (dyno disabled)
    engine->setSpeedControl(0.0);
    sim.m_dyno.m_enabled = false;
    sim.m_dyno.m_hold = false;
    runSimulationSteps(sim, 0.6, nullptr);
    drainAudio(sim, nullptr);

    return !outSamples.empty();
}

bool AudioExporter::generateThrottleBlip(
    PistonEngineSimulator &sim,
    std::vector<float> &outSamples)
{
    Engine *engine = sim.getEngine();
    if (!engine) return false;

    outSamples.clear();

    // 1. Ensure engine is running and stabilized at natural lowest idle (no dyno)
    ensureEngineRunning(sim);
    engine->setSpeedControl(0.0);
    runSimulationSteps(sim, 0.5, nullptr);
    drainAudio(sim, nullptr);

    // 2. Record 0.15s of quiet, stable natural idle lead-in
    engine->setSpeedControl(0.0);
    runSimulationSteps(sim, 0.15, &outSamples);

    // 3. Briefly tap R (snap to 1.0) — engine revs up freely under its own power
    engine->setSpeedControl(1.0);

    const double frameDt = 0.01;
    const double maxRevThreshold = engine->getRedline() * 0.90;

    for (double t = 0.0; t < 3.0; t += frameDt) {
        sim.startFrame(frameDt);
        while (sim.simulateStep()) {}
        sim.endFrame();
        drainAudio(sim, &outSamples);
        if (engine->getSpeed() >= maxRevThreshold) break;
    }

    // 4. Release throttle back to natural idle (0.0) — coast down naturally
    engine->setSpeedControl(0.0);

    // Target low base idle (~1050 RPM)
    const double idleTarget = units::rpm(1100);

    for (double t = 0.0; t < 5.0; t += frameDt) {
        sim.startFrame(frameDt);
        while (sim.simulateStep()) {}
        sim.endFrame();
        drainAudio(sim, &outSamples);
        if (t > 0.4 && engine->getSpeed() <= idleTarget) break;
    }

    // 5. Short quiet idle tail to settle smoothly
    runSimulationSteps(sim, 0.25, &outSamples);

    // Quick 50ms fade-in (~2200 samples) and 150ms fade-out (~6600 samples)
    WavWriter::applyEnvelopeFade(outSamples, 22050, 22050);

    return !outSamples.empty();
}


bool AudioExporter::generateDecelCrackle(
    PistonEngineSimulator &sim,
    int initialRpm,
    double durationSec,
    std::vector<float> &outSamples)
{
    Engine *engine = sim.getEngine();
    if (!engine) return false;

    outSamples.clear();

    // Ensure engine is running
    ensureEngineRunning(sim);

    // Hold at high RPM using dyno
    sim.m_dyno.m_enabled = true;
    sim.m_dyno.m_hold = true;
    sim.m_dyno.m_rotationSpeed = units::rpm(initialRpm);
    sim.m_dyno.m_maxTorque = 20000.0;
    engine->setSpeedControl(0.75);

    runSimulationSteps(sim, 0.8, nullptr);
    drainAudio(sim, nullptr);

    // Release dyno and chop throttle to idle
    sim.m_dyno.m_enabled = false;
    sim.m_dyno.m_hold = false;
    engine->setSpeedControl(0.0);

    // Record deceleration overrun crackle
    runSimulationSteps(sim, durationSec, &outSamples);

    // Apply envelope fade for smooth boundary transitions
    WavWriter::applyEnvelopeFade(outSamples, 8820, 13230);

    return !outSamples.empty();
}

void AudioExporter::writeManifest(
    const std::string &manifestPath,
    const VehicleExportConfig &vehicleConfig,
    const GlobalExportSettings &globalSettings,
    const std::vector<RenderedLoop> &loops,
    const std::vector<RenderedTransient> &transients)
{
    json j;
    j["vehicle_id"] = vehicleConfig.id;
    j["display_name"] = vehicleConfig.displayName;
    j["sample_rate"] = globalSettings.sampleRate;
    j["bits_per_sample"] = globalSettings.bitsPerSample;
    j["rpm_min"] = vehicleConfig.exportProfile.rpmMin;
    j["rpm_max"] = vehicleConfig.exportProfile.rpmMax;
    j["rpm_step"] = vehicleConfig.exportProfile.rpmStep;

    json loopsArray = json::array();
    for (const auto &loop : loops) {
        json item;
        item["rpm"] = loop.targetRpm;
        item["file"] = loop.fileName;
        item["duration_sec"] = loop.durationSec;
        item["loop_start_sample"] = loop.loopStartSample;
        item["loop_end_sample"] = loop.loopEndSample;
        loopsArray.push_back(item);
    }
    j["steady_loops"] = loopsArray;

    json transientsArray = json::array();
    for (const auto &tr : transients) {
        json item;
        item["type"] = tr.type;
        item["file"] = tr.fileName;
        item["duration_sec"] = tr.durationSec;
        transientsArray.push_back(item);
    }
    j["transients"] = transientsArray;

    std::ofstream out(manifestPath);
    if (out.is_open()) {
        out << j.dump(2) << std::endl;
    }
}

bool AudioExporter::exportVehicle(
    const VehicleExportConfig &vehicleConfig,
    const GlobalExportSettings &globalSettings,
    const std::vector<std::string> &searchPaths,
    const std::string &dataRoot,
    ProgressCallback *callback)
{
    std::cout << "\n=======================================================" << std::endl;
    std::cout << " Exporting Vehicle Audio: " << vehicleConfig.displayName << " (" << vehicleConfig.id << ")" << std::endl;
    std::cout << " Script: " << vehicleConfig.scriptPath << std::endl;
    std::cout << "=======================================================" << std::endl;

    // 1. Build search paths
    std::vector<piranha::IrPath> piranhaPaths;
    for (const auto &sp : vehicleConfig.extraSearchPaths) {
        piranhaPaths.push_back(sp);
    }
    for (const auto &sp : searchPaths) {
        piranhaPaths.push_back(sp);
    }
    piranhaPaths.push_back(dataRoot + "/assets");
    piranhaPaths.push_back(dataRoot + "/es");
    piranhaPaths.push_back(dataRoot);
    piranhaPaths.push_back(".");

    std::string scriptPathStr = vehicleConfig.scriptPath;
    std::filesystem::path resolvedScriptPath(scriptPathStr);
    if (!std::filesystem::exists(resolvedScriptPath)) {
        resolvedScriptPath = std::filesystem::path(dataRoot) / scriptPathStr;
    }

    std::cout << "Resolved script path: " << resolvedScriptPath.string() << std::endl;
    if (!std::filesystem::exists(resolvedScriptPath)) {
        std::cerr << "Error: Engine script not found: " << vehicleConfig.scriptPath << std::endl;
        return false;
    }

    // Add parent directory of script to piranha paths
    piranhaPaths.push_back(std::filesystem::absolute(resolvedScriptPath.parent_path()).string());

    // Read script content to detect defined public nodes
    std::string scriptContent;
    {
        std::ifstream sf(resolvedScriptPath);
        if (sf.is_open()) {
            std::stringstream ss;
            ss << sf.rdbuf();
            scriptContent = ss.str();
        }
    }

    std::string entryScriptContent;
    std::string detectedNode;

    // Check if script already defines 'public node main'
    bool hasMain = (scriptContent.find("public node main") != std::string::npos ||
                    scriptContent.find("node main") != std::string::npos);

    if (hasMain) {
        entryScriptContent =
            "import \"engine_sim.mr\"\n"
            "import \"themes/default.mr\"\n"
            "import \"" + resolvedScriptPath.filename().string() + "\"\n\n"
            "use_default_theme()\n"
            "main()\n";
    } else {
        // Look for public node <name>
        std::regex pubNodeRegex(R"(public\s+node\s+([a-zA-Z0-9_]+))");
        std::smatch match;
        if (std::regex_search(scriptContent, match, pubNodeRegex) && match.size() > 1) {
            detectedNode = match[1].str();
        } else {
            detectedNode = resolvedScriptPath.stem().string();
        }

        entryScriptContent =
            "import \"engine_sim.mr\"\n"
            "import \"themes/default.mr\"\n"
            "import \"" + resolvedScriptPath.filename().string() + "\"\n\n"
            "use_default_theme()\n"
            "public node main {\n"
            "    set_engine(" + detectedNode + "())\n"
            "}\n"
            "main()\n";
    }

    std::string tempEntryPath = "_temp_entry_" + vehicleConfig.id + ".mr";
    {
        std::ofstream tempFile(tempEntryPath);
        tempFile << entryScriptContent;
    }

    std::stringstream errorLog;
    es_script::Compiler compiler;
    compiler.initialize(piranhaPaths);

    bool compiled = compiler.compile(tempEntryPath, errorLog);

    if (!compiled) {
        std::cout << "Wrapper compilation failed. Error log:\n" << errorLog.str() << std::endl;
        compiler.destroy();
        compiler.initialize(piranhaPaths);
        std::stringstream fallbackLog;
        compiled = compiler.compile(resolvedScriptPath.string(), fallbackLog);
        std::cout << "Fallback direct compile result: " << compiled << std::endl;
        if (!compiled) {
            std::cerr << "Error: Failed to compile engine script: " << vehicleConfig.scriptPath << std::endl;
            std::cerr << fallbackLog.str() << std::endl;
            std::filesystem::remove(tempEntryPath);
            compiler.destroy();
            return false;
        }
    }

    es_script::Compiler::Output compiledOutput = compiler.execute();
    std::filesystem::remove(tempEntryPath);
    Engine *engine = compiledOutput.engine;
    Vehicle *vehicle = compiledOutput.vehicle;
    Transmission *transmission = compiledOutput.transmission;

    std::cout << "Engine pointer: " << engine << std::endl;
    if (!engine) {
        std::cerr << "Error: No engine found in script " << vehicleConfig.scriptPath << std::endl;
        compiler.destroy();
        return false;
    }

    if (vehicle == nullptr) {
        Vehicle::Parameters vehParams;
        vehParams.mass = units::mass(1597, units::kg);
        vehParams.diffRatio = 3.42;
        vehParams.tireRadius = units::distance(10, units::inch);
        vehParams.dragCoefficient = 0.25;
        vehParams.crossSectionArea = units::distance(60, units::inch) * units::distance(72, units::inch);
        vehParams.rollingResistance = 2000.0;
        vehicle = new Vehicle;
        vehicle->initialize(vehParams);
    }

    if (transmission == nullptr) {
        const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tParams;
        tParams.GearCount = 6;
        tParams.GearRatios = gearRatios;
        tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        transmission = new Transmission;
        transmission->initialize(tParams);
    }

    // 3. Initialize simulator via engine factory
    PistonEngineSimulator *simulator = static_cast<PistonEngineSimulator *>(engine->createSimulator(vehicle, transmission));
    simulator->setSimulationFrequency(engine->getSimulationFrequency());
    simulator->setTargetSynthesizerLatency(100.0); // Prevent real-time latency throttling in headless mode

    Synthesizer::AudioParameters audioParams = simulator->synthesizer().getAudioParameters();
    audioParams.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audioParams.airNoise = static_cast<float>(engine->getInitialNoise());
    audioParams.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
    simulator->synthesizer().setAudioParameters(audioParams);

    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse *response = engine->getExhaustSystem(i)->getImpulseResponse();
        if (!response) continue;

        std::filesystem::path irPath(response->getFilename());
        if (!std::filesystem::exists(irPath)) {
            irPath = std::filesystem::path(dataRoot) / response->getFilename();
            if (!std::filesystem::exists(irPath)) {
                irPath = std::filesystem::path(dataRoot) / "assets" / response->getFilename();
            }
        }

        ysAudioWaveFile waveFile;
        ysAudioFile::Error err = waveFile.OpenFile(irPath.string().c_str());
        if (err == ysAudioFile::Error::None) {
            waveFile.InitializeInternalBuffer(waveFile.GetSampleCount());
            waveFile.FillBuffer(0);
            waveFile.CloseFile();

            simulator->synthesizer().initializeImpulseResponse(
                reinterpret_cast<const int16_t *>(waveFile.GetBuffer()),
                waveFile.GetSampleCount(),
                static_cast<float>(response->getVolume()),
                i
            );

            waveFile.DestroyInternalBuffer();
        }
    }

    simulator->startAudioRenderingThread();

    // 4. Start & initialize engine
    if (callback) callback->onProgress(vehicleConfig.id, "Starting Engine", 0.05f);
    std::cout << "Starting and stabilizing engine..." << std::endl;
    initializeAndStartEngine(*simulator, engine);

    const std::string vehicleOutputDir = globalSettings.outputDir + "/" + vehicleConfig.id;
    std::filesystem::create_directories(vehicleOutputDir);

    WavWriter::WriteOptions wavOpts;
    wavOpts.sampleRate = globalSettings.sampleRate;
    wavOpts.bitsPerSample = globalSettings.bitsPerSample;
    wavOpts.normalize = true;
    wavOpts.targetPeakDbfs = globalSettings.normalizePeakDbfs;

    std::vector<RenderedLoop> renderedLoops;
    std::vector<RenderedTransient> renderedTransients;

    // 5. Export Steady RPM Loops
    if (vehicleConfig.exportProfile.exportSteadyRpm) {
        std::vector<int> targetRpms;
        if (!vehicleConfig.exportProfile.explicitRpms.empty()) {
            targetRpms = vehicleConfig.exportProfile.explicitRpms;
        } else {
            int step = vehicleConfig.exportProfile.rpmStep > 0 ? vehicleConfig.exportProfile.rpmStep : 500;
            for (int r = vehicleConfig.exportProfile.rpmMin; r <= vehicleConfig.exportProfile.rpmMax; r += step) {
                targetRpms.push_back(r);
            }
        }

        for (size_t idx = 0; idx < targetRpms.size(); ++idx) {
            int rpm = targetRpms[idx];
            float progress = 0.1f + 0.6f * (static_cast<float>(idx) / std::max(1.0f, static_cast<float>(targetRpms.size())));
            if (callback) callback->onProgress(vehicleConfig.id, "RPM " + std::to_string(rpm), progress);

            int cycles = vehicleConfig.exportProfile.cyclesPerLoop;
            if (vehicleConfig.exportProfile.minLoopDurationSec > 0.0 && rpm > 0) {
                double secPerCycle = 120.0 / static_cast<double>(rpm);
                cycles = std::max(cycles, static_cast<int>(std::ceil(vehicleConfig.exportProfile.minLoopDurationSec / secPerCycle)));
            }
            std::cout << "  -> Recording loop @ " << rpm << " RPM (" << cycles << " cycles, >=" << vehicleConfig.exportProfile.minLoopDurationSec << "s)..." << std::flush;

            std::vector<float> loopSamples;
            bool success = captureSteadyRpmLoop(
                *simulator,
                rpm,
                vehicleConfig.exportProfile.cyclesPerLoop,
                vehicleConfig.exportProfile.minLoopDurationSec,
                loopSamples);
            if (success && !loopSamples.empty()) {
                std::string fileName = "rpm_" + std::to_string(rpm) + ".wav";
                std::string filePath = vehicleOutputDir + "/" + fileName;

                WavWriter::WriteOptions loopWavOpts = wavOpts;
                loopWavOpts.loop.enabled = globalSettings.embedLoopMarkers;
                loopWavOpts.loop.startSample = 0;
                loopWavOpts.loop.endSample = static_cast<uint32_t>(loopSamples.size() - 1);

                WavWriter::writeWav(filePath, loopSamples, loopWavOpts);

                RenderedLoop rl;
                rl.targetRpm = rpm;
                rl.fileName = fileName;
                rl.samples = std::move(loopSamples);
                rl.durationSec = static_cast<double>(rl.samples.size()) / globalSettings.sampleRate;
                rl.loopStartSample = 0;
                rl.loopEndSample = static_cast<uint32_t>(rl.samples.size() - 1);
                renderedLoops.push_back(rl);

                std::cout << " OK (" << rl.durationSec << "s, " << rl.samples.size() << " samples)\n";
            } else {
                std::cout << " FAILED\n";
            }
        }
    }

    // 6. Export Transient Sounds
    // A. Engine Start (from silent -> rev to 2000 RPM -> decel to idle)
    if (vehicleConfig.exportProfile.exportStarter) {
        if (callback) callback->onProgress(vehicleConfig.id, "Engine Start", 0.75f);
        std::cout << "  -> Recording engine start..." << std::flush;
        std::vector<float> samples;
        if (generateEngineStart(*simulator, samples)) {
            std::string fileName = "engine_start.wav";
            WavWriter::writeWav(vehicleOutputDir + "/" + fileName, samples, wavOpts);
            RenderedTransient rt;
            rt.type = "engine_start";
            rt.fileName = fileName;
            rt.durationSec = static_cast<double>(samples.size()) / globalSettings.sampleRate;
            renderedTransients.push_back(rt);
            std::cout << " OK (" << rt.durationSec << "s)\n";
        }
    }

    // C. Rev Limiter
    if (vehicleConfig.exportProfile.exportRevLimiter) {
        if (callback) callback->onProgress(vehicleConfig.id, "Rev Limiter", 0.85f);
        std::cout << "  -> Recording rev limiter bounce..." << std::flush;
        std::vector<float> samples;
        if (generateRevLimiter(*simulator, 2.5, samples)) {
            std::string fileName = "rev_limiter.wav";
            WavWriter::writeWav(vehicleOutputDir + "/" + fileName, samples, wavOpts);
            RenderedTransient rt;
            rt.type = "rev_limiter";
            rt.fileName = fileName;
            rt.durationSec = static_cast<double>(samples.size()) / globalSettings.sampleRate;
            renderedTransients.push_back(rt);
            std::cout << " OK (" << rt.durationSec << "s)\n";
        }
    }

    // D. Throttle Blip
    if (vehicleConfig.exportProfile.exportRevBlip) {
        if (callback) callback->onProgress(vehicleConfig.id, "Throttle Blip", 0.90f);
        std::cout << "  -> Recording throttle blip..." << std::flush;
        std::vector<float> samples;
        if (generateThrottleBlip(*simulator, samples)) {
            std::string fileName = "rev_blip.wav";
            WavWriter::writeWav(vehicleOutputDir + "/" + fileName, samples, wavOpts);
            RenderedTransient rt;
            rt.type = "rev_blip";
            rt.fileName = fileName;
            rt.durationSec = static_cast<double>(samples.size()) / globalSettings.sampleRate;
            renderedTransients.push_back(rt);
            std::cout << " OK (" << rt.durationSec << "s)\n";
        }
    }

    // E. Decel Crackle
    if (vehicleConfig.exportProfile.exportDecelCrackle) {
        if (callback) callback->onProgress(vehicleConfig.id, "Decel Crackle", 0.95f);
        std::cout << "  -> Recording decel crackle..." << std::flush;
        std::vector<float> samples;
        int decelRpm = std::max(4000, vehicleConfig.exportProfile.rpmMax - 1000);
        if (generateDecelCrackle(*simulator, decelRpm, 2.0, samples)) {
            std::string fileName = "decel_crackle.wav";
            WavWriter::writeWav(vehicleOutputDir + "/" + fileName, samples, wavOpts);
            RenderedTransient rt;
            rt.type = "decel_crackle";
            rt.fileName = fileName;
            rt.durationSec = static_cast<double>(samples.size()) / globalSettings.sampleRate;
            renderedTransients.push_back(rt);
            std::cout << " OK (" << rt.durationSec << "s)\n";
        }
    }

    // 7. Write Manifest
    std::string manifestPath = vehicleOutputDir + "/manifest.json";
    writeManifest(manifestPath, vehicleConfig, globalSettings, renderedLoops, renderedTransients);
    std::cout << "Wrote Godot manifest: " << manifestPath << "\n";

    // 8. Cleanup
    simulator->releaseSimulation();
    delete simulator;
    delete vehicle;
    delete transmission;
    engine->destroy();
    delete engine;
    compiler.destroy();

    if (callback) callback->onProgress(vehicleConfig.id, "Complete", 1.0f);
    return true;
}

bool AudioExporter::exportRecipe(
    const ExportRecipe &recipe,
    const std::string &dataRoot,
    ProgressCallback *callback)
{
    std::cout << "\n=======================================================\n";
    std::cout << " Starting Batch Engine Sound Export\n";
    std::cout << " Vehicles to export: " << recipe.vehicles.size() << "\n";
    std::cout << " Output destination: " << recipe.globalSettings.outputDir << "\n";
    std::cout << "=======================================================\n";

    for (size_t i = 0; i < recipe.vehicles.size(); ++i) {
        const auto &vc = recipe.vehicles[i];
        exportVehicle(vc, recipe.globalSettings, recipe.defaultSearchPaths, dataRoot, callback);
    }

    std::cout << "\nBatch Audio Export Complete!\n";
    return true;
}

