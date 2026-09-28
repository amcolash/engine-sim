# Audio Export & Simulation Pipeline Documentation

This document describes the engine audio synthesis, export, post-processing, and synchronization pipeline in `engine-sim`. It covers the batch exporter architecture, JSON export recipes, steady RPM loop generation, transient SFX simulation procedures, active RMS loudness normalization, and the intelligent diff/sync tool for game engine integration.

---

## 1. Overview & Architecture

`engine-sim` features a headless, multi-threaded audio export system designed to render physics-accurate engine audio from `.mr` script models directly into high-fidelity `.wav` assets.

```
                  +-----------------------------------+
                  |      JSON Export Recipe           |
                  |  (e.g. recipes/game_vehicles.json)|
                  +-----------------+-----------------+
                                    |
                                    v
+-----------------------------------+-----------------------------------+
|               Headless Audio Exporter (C++ Multi-Threaded)            |
|                                                                       |
|  +---------------------------+     +-------------------------------+  |
|  | Steady RPM Loops          |     | Transient SFX                 |  |
|  | - Dyno RPM stabilization  |     | - engine_start                |  |
|  | - Exact 4-stroke cycles   |     | - rev_blip                    |  |
|  | - Embedded smpl markers   |     | - rev_limiter                 |  |
|  | - Boundary micro-crossfade|     | - decel_crackle               |  |
|  +---------------------------+     +-------------------------------+  |
|                                                                       |
|                  WavWriter (RMS Normalization & Soft Limiter)         |
+-----------------------------------+-----------------------------------+
                                    |
                                    v
                  +-----------------------------------+
                  | assets/audio/engines/<vehicle_id>/|
                  +-----------------+-----------------+
                                    |
                                    v
+-----------------------------------+-----------------------------------+
|               Intelligent Sync Tool (scripts/sync_audio.py)           |
|                                                                       |
|  - In-place active RMS loudness normalization (-12 dBFS)              |
|  - Spectral cosine similarity & RMS delta perceptual diff             |
|  - Zero-churn sync to game repo (Godot / Unity / custom)              |
+-----------------------------------------------------------------------+
```

### CLI & Justfile Commands

| Command | Description |
|---|---|
| `just export-audio [args]` | Exports all vehicles in `recipes/game_vehicles.json` (full batch) and runs sync. |
| `just export-sfx <type> [args]` | Exports specific SFX only (e.g. `engine_start`, `rev_blip`, `rev_limiter`, `decel_crackle`, `all`). Supports `--vehicle <id>`. |
| `just export-fast [args]` | Exports fast smoke-test vehicle audio from `recipes/fast_test.json`. |
| `just export-engine <script.mr> [args]` | Directly exports a single `.mr` script. |
| `just sync-audio [args]` | Runs post-process normalization and perceptual diff sync to Godot without re-rendering. |

---

## 2. Recipe System (`recipes/*.json`)

Export configurations are stored as JSON recipes. The primary game vehicle recipe is located at [`recipes/game_vehicles.json`](file:///home/amcolash/Dev/engine-sim/recipes/game_vehicles.json).

### Recipe Structure

```json
{
  "global_settings": {
    "output_dir": "assets/audio/engines",
    "sample_rate": 44100,
    "bits_per_sample": 16,
    "normalize_peak_dbfs": -1.0,
    "embed_loop_markers": true,
    "default_cycles_per_loop": 6,
    "default_min_loop_duration_sec": 1.0
  },
  "search_paths": ["assets", "assets/engines", "assets/engines/custom", "part-library"],
  "vehicles": [
    {
      "id": "trench",
      "display_name": "Trench (MUSCLE) — Classic Pony V8",
      "script_path": "assets/engines/atg-video-2/07_gm_ls.mr",
      "export_profile": {
        "rpm_min": 900,
        "rpm_max": 6200,
        "rpm_step": 500,
        "cycles_per_loop": 6,
        "min_loop_duration_sec": 1.0,
        "export_steady_rpm": true,
        "export_starter": true,
        "export_rev_limiter": true,
        "export_rev_blip": true,
        "export_decel_crackle": true
      }
    }
  ]
}
```

### Configuration Fields

- **`global_settings`**:
  - `output_dir`: Destination directory for exported engine folders.
  - `sample_rate`: Output audio sample rate in Hz (default: `44100`).
  - `bits_per_sample`: Bit depth (default: `16` PCM, supports `24` PCM).
  - `normalize_peak_dbfs`: Maximum peak ceiling in dBFS (default: `-1.0`).
  - `embed_loop_markers`: If `true`, writes standard RIFF `smpl` chunk loop points into WAV headers.
  - `default_cycles_per_loop`: Minimum complete 4-stroke combustion cycles per steady loop.
  - `default_min_loop_duration_sec`: Minimum duration threshold in seconds per loop file.
- **`vehicles[]`**:
  - `id`: Unique identifier used for folder naming and game integration.
  - `script_path`: Path to the `.mr` engine simulation script.
  - `export_profile`:
    - `rpm_min`, `rpm_max`, `rpm_step`: Range and interval for steady loops.
    - `explicit_rpms`: Optional explicit array of RPM values (overrides min/max/step).
    - `export_steady_rpm`: Toggles generation of steady RPM loop files (`rpm_<RPM>.wav`).
    - `export_starter`: Toggles `engine_start.wav`.
    - `export_rev_limiter`: Toggles `rev_limiter.wav`.
    - `export_rev_blip`: Toggles `rev_blip.wav`.
    - `export_decel_crackle`: Toggles `decel_crackle.wav`.

### Output Manifest (`manifest.json`)

Each vehicle export produces a `manifest.json` inside its folder describing all exported assets:

```json
{
  "vehicle_id": "monarch",
  "display_name": "Monarch (MOBIL) — Merlin 27L Supercharged V12",
  "sample_rate": 44100,
  "bits_per_sample": 16,
  "rpm_min": 800,
  "rpm_max": 3800,
  "rpm_step": 400,
  "steady_loops": [
    {
      "rpm": 800,
      "file": "rpm_800.wav",
      "duration_sec": 1.05,
      "loop_start_sample": 0,
      "loop_end_sample": 46304
    }
  ],
  "transients": [
    { "type": "engine_start", "file": "engine_start.wav", "duration_sec": 1.95 },
    { "type": "rev_limiter", "file": "rev_limiter.wav", "duration_sec": 2.52 },
    { "type": "rev_blip", "file": "rev_blip.wav", "duration_sec": 4.54 },
    { "type": "decel_crackle", "file": "decel_crackle.wav", "duration_sec": 2.00 }
  ]
}
```

---

## 3. Steady RPM Loops Export

Steady loops represent sustained engine audio at discrete RPM increments (e.g. 900 to 6500 RPM in 500 RPM steps).

### Generation Process
1. **RPM Hold & Stabilization**: The engine dynamometer (`sim.m_dyno`) engages in speed-hold mode at the target RPM. Throttle is modulated dynamically to overcome friction and pumping losses. The engine runs unrecorded for `0.8s` to settle intake/exhaust manifold pressure waves into an exact periodic steady state.
2. **Cycle Length Calculation**: For a four-stroke engine, one complete engine cycle requires 2 full crankshaft rotations ($720^\circ$).
   $$\text{Duration per cycle} = \frac{2 \times 60}{\text{RPM}} = \frac{120}{\text{RPM}}\text{ seconds}$$
3. **Exact Cycle Capture**: The recording captures exactly $N$ integer engine cycles (where $N \ge \text{cycles\_per\_loop}$ and $N \times T_{\text{cycle}} \ge \text{min\_loop\_duration\_sec}$).
4. **Micro-Crossfade**: A 64-sample cosine micro-crossfade is applied between the loop tail and head to guarantee phase continuity and zero zero-crossing clicks.
5. **RIFF `smpl` Chunk Embedding**: A standard WAV `smpl` sampler metadata chunk is written with `loop_start = 0` and `loop_end = num_samples - 1`. Game engines (Godot, Unity, FMOD, Wwise) read this chunk directly for seamless hardware-accelerated looping.

---

## 4. Transient Sound Effects Simulation (SFX)

All transient sound effects are generated by directly driving the physical engine simulation through realistic mechanical sequences.

### A. Engine Start (`engine_start.wav`)
Simulates starting the engine from a dead stop (0 RPM) through starter cranking, ignition catch, rev flair, and settling into natural idle.

```
Time:     0.0s        0.10s                         0.95s                        1.95s
State:   [ Dead Stop ] [ Starter Motor Cranks (0.85s) ] [ Natural Idle Run (1.0s) ]
RPM:      0 RPM ------> Cranking (150-250 RPM) -------> Catches & Flairs -> Idle (~900 RPM)
Ignition: OFF --------> ON ------------------------------------------------------> ON
Throttle: 0.0 --------> 0.0 -----------------------------------------------------> 0.0 (Idle)
Fade:     Silence       Active Cranking & Firing        200ms Envelope Fade-Out
```

1. **Dead Stop**: Dyno brake brings the crankshaft to 0 RPM with ignition disabled; synth convolution ringout buffers are drained to absolute silence.
2. **Pre-crank Silence**: Records 0.10s of silence before key-turn.
3. **Starter Engagement**: Starter motor engages at target `starter_speed` (150–250 RPM) with `starter_torque` (200–450 lb-ft). Ignition is switched ON with idle throttle (`speed_control = 0.0`).
4. **Crank & Fire (0.85s)**: The engine cranks from standstill, builds manifold vacuum, draws fuel/air mixture, catches on first compression strokes, and accelerates past starter speed.
5. **Starter Disengagement**: Starter motor is released.
6. **Idle Settle (1.0s)**: Engine runs freely under its own power at natural idle throttle.
7. **Smooth Fade-Out**: A 200ms cosine fade-out is applied at the tail.

### B. Rev Limiter (`rev_limiter.wav`)
Simulates bouncing aggressively against the hard electronic rev limiter.

1. **Warmup**: Dyno holds engine at redline with Wide Open Throttle (`speed_control = 1.0`) for 0.5s.
2. **Bouncing Cycle (2.5s duration)**:
   - **Surge Phase (60ms)**: Dyno target set to `redline`, throttle at 1.0 (WOT power surge).
   - **Cut Phase (60ms)**: Dyno target set to `redline - 500 RPM`, throttle chopped to 0.10 (simulating ignition cut / fuel cut overrun dip).
   - Cycles at ~8.3 Hz (120ms period), producing sharp, aggressive exhaust backfire pops.
3. **Smoothing**: 64-sample micro-crossfade and 256-sample envelope fade.

### C. Throttle Blip (`rev_blip.wav`)
Simulates snapping the throttle wide open from idle in neutral and letting the engine freely rev up and coast down.

```
RPM ^
    |                   /---\  (90% Redline)
    |                  /     \
    |                 /       \
    |                /         \
    |  (Idle)  _____/           \________ (Coast to Idle)
    +----------------------------------------------------> Time
       [ 0.15s ][ WOT Blip (1.0) ][ Free Decel (0.0) ][ 0.25s Tail ]
```

1. **Idle Lead-in**: Engine stabilizes at natural idle (0.5s); records 0.15s quiet idle lead-in.
2. **WOT Tap**: Speed control snaps to `1.0` (WOT). The engine accelerates under its own flywheel inertia until reaching 90% of redline RPM.
3. **Throttle Chop**: Speed control snaps back to `0.0`. Dyno is completely disengaged; engine coasts down under internal friction and pumping resistance until settling below ~1100 RPM.
4. **Settle Tail**: Records 0.25s of natural idle tail followed by smooth envelope fading.

### D. Deceleration Crackle (`decel_crackle.wav`)
Simulates high-RPM off-throttle overrun exhaust crackles, pops, and gurgles.

1. **High RPM Hold**: Dyno holds engine at `rpm_max - 1000` (min 4000 RPM) at 75% throttle for 0.8s.
2. **Throttle Cut & Coast**: Dyno is released and throttle is chopped to 0.0 (idle).
3. **Overrun Recording (2.0s)**: Records unburnt fuel burning in exhaust runners during engine braking deceleration.
4. **Envelope Fade**: Smooth fade-in (8820 samples) and fade-out (13230 samples).

---

## 5. Audio Normalization & Peak Limiting Pipeline

### Problem with Pure Peak Normalization
Raw internal combustion synthesis produces occasional high-energy single-sample transient spikes (such as ignition onset or starter clunks). Standard peak normalization scales the entire audio file based on the highest single peak. If a file has one spike at $10.0$ and steady combustion at $0.1$, peak normalization scales everything down by $10\times$, resulting in an active audio level of $-40\text{ dBFS}$ (barely audible).

### Dual-Layer Loudness Normalization Solution
The pipeline employs active RMS loudness targeting combined with a $C^1$-continuous soft-knee saturation limiter:

```
Input Audio
    |
    v
[ Calculate Active RMS ] (Filters out silent tails & noise below -40 dB relative to peak)
    |
    v
[ Calculate Target Gain ] (Target: -12.0 dBFS RMS, gain bound <= 3.0x max peak)
    |
    v
[ Soft-Knee Saturation Limiter ]
    |-- If |x| <= Knee (0.75 * targetPeak):  x (100% bit-transparent & linear)
    \-- If |x| >  Knee:  knee + delta * tanh((|x| - knee) / delta) -> asymptotic ceiling at -1.0 dBFS
    |
    v
Output WAV (Punchy, loud, consistent across vehicles, zero digital clipping)
```

1. **Active RMS Calculation**:
   $$\text{Threshold} = \text{max\_peak} \times 10^{-40 / 20} = 0.01 \times \text{max\_peak}$$
   $$\text{Active RMS} = \sqrt{\frac{1}{M}\sum_{|x_i| \ge \text{Threshold}} x_i^2}$$
2. **Gain Calculation**:
   $$\text{Gain} = \frac{10^{-12.0 / 20}}{\text{Active RMS}} \approx \frac{0.2512}{\text{Active RMS}}$$
3. **Soft-Knee Peak Limiting Transfer Function**:
   For $\text{Knee} = 0.75 \times \text{targetPeak}$ and $\Delta = \text{targetPeak} - \text{Knee}$:
   $$y(x) = \begin{cases} x & |x| \le \text{Knee} \\ \operatorname{sgn}(x)\left(\text{Knee} + \Delta \tanh\left(\frac{|x| - \text{Knee}}{\Delta}\right)\right) & |x| > \text{Knee} \end{cases}$$
   Because $\tanh(0) = 0$ and $\tanh'(0) = 1$, the transition at the knee is perfectly smooth ($C^1$ continuous), introducing zero harsh clipping or harmonic distortion.

This normalization is implemented in:
- **C++ Exporter**: [`src/wav_writer.cpp`](file:///home/amcolash/Dev/engine-sim/src/wav_writer.cpp#L29) (`WavWriter::normalizeAudio`)
- **Post-Process Sync Pipeline**: [`scripts/sync_audio.py`](file:///home/amcolash/Dev/engine-sim/scripts/sync_audio.py#L138) (`normalize_wav_file`, invoked automatically via `just sync-audio`, `just export-audio`, and `just export-sfx`)

---

## 6. Intelligent Diff & Sync Tool

The sync pipeline automatically copies exported audio from `assets/audio/engines` to the game repository (e.g. `/home/amcolash/Godot/drag-race/assets/engine`).

### Perceptual Equivalence Analysis
To prevent massive Git binary churn when re-exporting audio batches, the sync tool performs perceptual acoustic comparison:

1. **Sample Rate & Duration**: Verifies sample rate match and checks that duration delta is $< 8\%$.
2. **Active RMS Energy Delta**: Verifies that active RMS energy change is $< 3.5\text{ dB}$.
3. **Combustion Peak Frequency**: Computes FFT magnitude spectrum in the 20 Hz – 10,000 Hz range and verifies dominant combustion firing frequency matches within $6\%$.
4. **Logarithmic Mel Spectral Cosine Similarity**: Bins frequency energy into 64 logarithmically spaced bands ($20\text{ Hz} \to 10\text{ kHz}$) and computes cosine similarity:
   $$\text{Similarity} = \frac{\mathbf{E}_{\text{exist}} \cdot \mathbf{E}_{\text{new}}}{\|\mathbf{E}_{\text{exist}}\| \|\mathbf{E}_{\text{new}}\|}$$
   If $\text{Similarity} \ge 98.5\%$, the file is marked **Untouched (Identical)** and skipped from disk write, generating **0 Git diff**.

### Recommended Justfile Commands
```bash
# Standard sync with active RMS normalization across all vehicles
just sync-audio

# Sync a specific vehicle only
just sync-audio --vehicle nomad

# Force overwrite all files regardless of match
just sync-audio --force

# Dry run (compare and report without writing)
just sync-audio --dry-run
```
