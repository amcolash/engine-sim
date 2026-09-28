#!/usr/bin/env python3
"""
Intelligent Audio Diff & Sync Tool for Engine-Sim -> Game Repository.

Compares newly generated .wav engine audio against tracked game repository files.
If a file is acoustically identical (within perceptual spectral and RMS tolerance),
it skips copying to prevent unnecessary binary churn in Git.
"""

import argparse
import os
import shutil
import sys
import wave
import numpy as np

def load_wav(file_path):
    """Loads a WAV file and returns (sample_rate, samples_float_array)."""
    with wave.open(file_path, 'rb') as wf:
        n_channels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        framerate = wf.getframerate()
        n_frames = wf.getnframes()
        raw_bytes = wf.readframes(n_frames)

    if sampwidth == 2:
        dtype = np.int16
        scale = 32768.0
    elif sampwidth == 3:
        # 24-bit PCM
        raw_array = np.frombuffer(raw_bytes, dtype=np.uint8)
        # Reshape to 3 bytes per sample
        samples_24 = raw_array.reshape(-1, 3)
        # Pad with 0 for sign extension to int32
        padded = np.pad(samples_24, ((0, 0), (1, 0)), mode='constant')
        samples_32 = padded.view('<i4') >> 8
        return framerate, (samples_32.astype(np.float32) / 8388608.0).flatten()
    elif sampwidth == 4:
        dtype = np.int32
        scale = 2147483648.0
    else:
        return None, None

    samples = np.frombuffer(raw_bytes, dtype=dtype).astype(np.float32) / scale
    if n_channels > 1:
        samples = samples.reshape(-1, n_channels).mean(axis=1)

    return framerate, samples

def compute_spectral_features(samples, sample_rate, n_bands=64):
    """Computes log-frequency energy distribution and dominant combustion frequency."""
    if len(samples) == 0:
        return np.zeros(n_bands), 0.0

    # Apply Hann window
    windowed = samples * np.hanning(len(samples))
    fft_mag = np.abs(np.fft.rfft(windowed))
    freqs = np.fft.rfftfreq(len(samples), d=1.0 / sample_rate)

    # Focus on audio range 20 Hz - 10000 Hz
    valid_idx = (freqs >= 20.0) & (freqs <= 10000.0)
    valid_freqs = freqs[valid_idx]
    valid_mag = fft_mag[valid_idx]

    if len(valid_mag) == 0 or np.max(valid_mag) == 0:
        return np.zeros(n_bands), 0.0

    # Dominant peak frequency
    peak_freq = valid_freqs[np.argmax(valid_mag)]

    # Logarithmic frequency binning (similar to constant-Q / mel bands)
    log_bins = np.logspace(np.log10(20.0), np.log10(10000.0), n_bands + 1)
    band_energies = np.zeros(n_bands, dtype=np.float32)

    for i in range(n_bands):
        mask = (valid_freqs >= log_bins[i]) & (valid_freqs < log_bins[i + 1])
        if np.any(mask):
            band_energies[i] = np.mean(valid_mag[mask] ** 2)

    # Normalize band energies
    norm = np.linalg.norm(band_energies)
    if norm > 1e-12:
        band_energies /= norm

    return band_energies, peak_freq

def compare_audio(existing_path, new_path, similarity_threshold=0.985, max_rms_delta_db=3.5):
    """
    Compares two WAV files for perceptual equivalence.
    Returns (is_match, reason, similarity_score, rms_delta_db).
    """
    if not os.path.exists(existing_path):
        return False, "New file (destination does not exist)", 0.0, 0.0

    try:
        sr_exist, samples_exist = load_wav(existing_path)
        sr_new, samples_new = load_wav(new_path)
    except Exception as e:
        return False, f"Failed to read WAV: {e}", 0.0, 0.0

    if sr_exist != sr_new:
        return False, f"Sample rate mismatch ({sr_exist} vs {sr_new})", 0.0, 0.0

    # Duration comparison (allow up to 8% difference for cycle-length adjustments)
    dur_exist = len(samples_exist) / sr_exist
    dur_new = len(samples_new) / sr_new
    dur_delta_ratio = abs(dur_exist - dur_new) / max(dur_exist, dur_new, 1e-6)

    if dur_delta_ratio > 0.08:
        return False, f"Duration changed ({dur_exist:.2f}s -> {dur_new:.2f}s)", 0.0, 0.0

    # RMS Energy comparison (3.5 dB accounts for peak-normalization jitter across stochastic combustion)
    rms_exist = np.sqrt(np.mean(samples_exist ** 2)) + 1e-9
    rms_new = np.sqrt(np.mean(samples_new ** 2)) + 1e-9
    rms_delta_db = abs(20.0 * np.log10(rms_new / rms_exist))

    if rms_delta_db > max_rms_delta_db:
        return False, f"RMS energy delta too high ({rms_delta_db:.2f} dB > {max_rms_delta_db} dB)", 0.0, rms_delta_db

    # Spectral analysis
    bands_exist, peak_exist = compute_spectral_features(samples_exist, sr_exist)
    bands_new, peak_new = compute_spectral_features(samples_new, sr_new)

    # Peak frequency check
    if peak_exist > 0 and peak_new > 0:
        peak_delta_ratio = abs(peak_exist - peak_new) / peak_exist
        if peak_delta_ratio > 0.06:
            return False, f"Combustion peak frequency changed ({peak_exist:.1f}Hz -> {peak_new:.1f}Hz)", 0.0, rms_delta_db

    # Cosine similarity of frequency energy bands
    cosine_sim = float(np.dot(bands_exist, bands_new))

    if cosine_sim < similarity_threshold:
        return False, f"Spectral profile changed (similarity {cosine_sim * 100:.1f}% < {similarity_threshold * 100:.1f}%)", cosine_sim, rms_delta_db

    return True, f"Acoustically identical ({cosine_sim * 100:.1f}% match, {rms_delta_db:.2f} dB delta)", cosine_sim, rms_delta_db

def sync_directories(src_dir, dst_dir, vehicle_filter=None, threshold=0.985, max_rms_delta=3.5, force=False, dry_run=False):
    """Syncs audio from src_dir to dst_dir, skipping perceptually identical files."""
    if not os.path.exists(src_dir):
        print(f"Error: Source directory '{src_dir}' not found.")
        return 1

    os.makedirs(dst_dir, exist_ok=True)

    vehicles = [d for d in os.listdir(src_dir) if os.path.isdir(os.path.join(src_dir, d))]
    vehicles.sort()

    if vehicle_filter:
        vehicles = [v for v in vehicles if v.lower() == vehicle_filter.lower()]
        if not vehicles:
            print(f"Error: Vehicle '{vehicle_filter}' not found in {src_dir}")
            return 1

    print("=" * 70)
    print(" Intelligent Audio Diff & Repository Sync")
    print(f" Source:        {src_dir}")
    print(f" Destination:   {dst_dir}")
    print(f" Vehicles:      {len(vehicles)} vehicle(s)")
    print(f" Spectral Tol:  {threshold * 100:.1f}% cosine similarity")
    print(f" Max RMS Delta: {max_rms_delta:.1f} dB")
    if force:
        print(" Mode:          FORCE OVERWRITE ALL")
    elif dry_run:
        print(" Mode:          DRY RUN (no files modified)")
    print("=" * 70)

    total_files = 0
    updated_files = 0
    skipped_files = 0

    for v in vehicles:
        v_src = os.path.join(src_dir, v)
        v_dst = os.path.join(dst_dir, v)
        os.makedirs(v_dst, exist_ok=True)

        wav_files = [f for f in os.listdir(v_src) if f.endswith('.wav')]
        wav_files.sort()

        v_updated = 0
        v_skipped = 0

        for wf in wav_files:
            total_files += 1
            src_file = os.path.join(v_src, wf)
            dst_file = os.path.join(v_dst, wf)

            if force or not os.path.exists(dst_file):
                is_match = False
                reason = "Forced update" if force else "New file"
                sim = 0.0
                rms_d = 0.0
            else:
                is_match, reason, sim, rms_d = compare_audio(dst_file, src_file, similarity_threshold=threshold, max_rms_delta_db=max_rms_delta)

            if is_match:
                v_skipped += 1
                skipped_files += 1
            else:
                v_updated += 1
                updated_files += 1
                print(f"  [{v}] UPDATE {wf:20s} -> {reason}")
                if not dry_run:
                    shutil.copy2(src_file, dst_file)

        # Copy any non-wav files (manifests, metadata) if changed
        for extra in os.listdir(v_src):
            if not extra.endswith('.wav'):
                s_extra = os.path.join(v_src, extra)
                d_extra = os.path.join(v_dst, extra)
                should_copy = False
                if not os.path.exists(d_extra):
                    should_copy = True
                else:
                    with open(s_extra, 'rb') as f1, open(d_extra, 'rb') as f2:
                        if f1.read() != f2.read():
                            should_copy = True
                if should_copy and not dry_run:
                    shutil.copy2(s_extra, d_extra)

        status_str = f"[{v}] {v_skipped}/{len(wav_files)} untouched (identical)"
        if v_updated > 0:
            status_str += f", {v_updated} updated"
        print(status_str)

    print("-" * 70)
    print(f" Summary: {total_files} total files analyzed")
    print(f"   Untouched (Identical - 0 Git diff): {skipped_files}")
    print(f"   Updated / Copied:                   {updated_files}")
    print("=" * 70)
    return 0

def main():
    parser = argparse.ArgumentParser(description="Acoustic Diff & Sync Tool for Engine-Sim to Game Repo")
    parser.add_argument("--src", default="assets/audio/engines", help="Source audio directory")
    parser.add_argument("--dst", default="/home/amcolash/Godot/drag-race/assets/engine", help="Destination game repo directory")
    parser.add_argument("--vehicle", default=None, help="Sync specific vehicle only (e.g. 'stratus')")
    parser.add_argument("--threshold", type=float, default=0.985, help="Similarity threshold (0.0 to 1.0, default 0.985)")
    parser.add_argument("--max-rms-delta", type=float, default=3.5, help="Max RMS delta in dB (default 3.5 dB)")
    parser.add_argument("--force", action="store_true", help="Force overwrite all files regardless of match")
    parser.add_argument("--dry-run", action="store_true", help="Perform comparison without copying files")

    args = parser.parse_args()
    return sync_directories(
        src_dir=args.src,
        dst_dir=args.dst,
        vehicle_filter=args.vehicle,
        threshold=args.threshold,
        max_rms_delta=args.max_rms_delta,
        force=args.force,
        dry_run=args.dry_run
    )

if __name__ == "__main__":
    sys.exit(main())
