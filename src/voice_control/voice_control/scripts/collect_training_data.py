#!/usr/bin/env python3
"""Interactive script to collect training WAV files for the VQ codebooks.

For each word in VOCABULARY:
    1. Prints "Say: <word>"
    2. Records 15 utterances of ~1.5 s each
    3. Saves them as codebooks/training/<word>/<word>_NN.wav

Usage::

    python3 scripts/collect_training_data.py [--output-dir PATH]
                                              [--utterances N]
                                              [--duration S]
                                              [--sample-rate HZ]

Requires pyaudio and scipy to be installed.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np


def _save_wav(path: str, data: np.ndarray, sample_rate: int) -> None:
    import scipy.io.wavfile as wavfile  # noqa: PLC0415

    int_data = (data * 32767).astype(np.int16)
    wavfile.write(path, sample_rate, int_data)


def _record_utterance(recorder, duration_s: float) -> np.ndarray:
    """Record one utterance of fixed duration."""
    return recorder.record_segment(duration_s)


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Collect voice training data for the AMR VQ codebooks.'
    )
    parser.add_argument(
        '--output-dir',
        default=os.path.join(os.path.dirname(__file__), '..', 'codebooks', 'training'),
        help='Root directory where recordings are saved.',
    )
    parser.add_argument('--utterances', type=int, default=15, help='Recordings per word.')
    parser.add_argument('--duration', type=float, default=1.5, help='Duration per recording (s).')
    parser.add_argument('--sample-rate', type=int, default=16000, help='Sample rate in Hz.')
    args = parser.parse_args()

    # Late import so failure message is readable
    try:
        from voice_control.command_parser import VOCABULARY
        from voice_control.record import AudioRecorder
    except ImportError as exc:
        print(f'ERROR: Could not import voice_control modules: {exc}', file=sys.stderr)
        print('Make sure the package is built and sourced.', file=sys.stderr)
        sys.exit(1)

    output_dir = os.path.realpath(args.output_dir)
    recorder = AudioRecorder(sample_rate=args.sample_rate, channels=1)

    print(f'\nCollecting {args.utterances} utterance(s) per word  '
          f'({args.duration:.1f} s each, {args.sample_rate} Hz)')
    print(f'Output directory: {output_dir}\n')

    for word in VOCABULARY:
        word_dir = os.path.join(output_dir, word)
        os.makedirs(word_dir, exist_ok=True)

        print(f'─── Word: "{word}" ───────────────────────────────────────')
        input(f'  Press ENTER to start recording "{word}" ...')

        for i in range(args.utterances):
            print(f'  [{i + 1:02d}/{args.utterances}] Say: {word!r}  (recording {args.duration:.1f} s ...)',
                  end=' ', flush=True)

            try:
                audio = _record_utterance(recorder, args.duration)
            except RuntimeError as exc:
                print(f'\n  ERROR: {exc}')
                print('  Install pyaudio:  pip install pyaudio')
                sys.exit(1)

            filename = f'{word}_{i:02d}.wav'
            filepath = os.path.join(word_dir, filename)
            _save_wav(filepath, audio, args.sample_rate)
            print(f'saved → {filename}')

        print(f'  Finished "{word}"\n')

    print('All words collected.  Now train codebooks with:')
    print('  ros2 run voice_control train')
    print('or:')
    print('  python3 -m voice_control.train')


if __name__ == '__main__':
    main()
