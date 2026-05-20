"""Grammar-based command parser for the AMR voice interface.

Maps recognised word sequences to structured robot commands.
Vocabulary matches the 10 words trained in the HMM system.
"""

from __future__ import annotations

VOCABULARY: list[str] = [
    'start', 'stop', 'left', 'right', 'forward',
    'backward', 'lift', 'leave', 'break', 'continue',
]

# ── Grammar rules ─────────────────────────────────────────────────────────────
# Each rule: (pattern_fn, result_fn)

_RULES: list[tuple] = [
    # Single-word control commands
    (lambda w: w == ['start'],    lambda w: {'cmd': 'start'}),
    (lambda w: w == ['stop'],     lambda w: {'cmd': 'stop'}),
    (lambda w: w == ['break'],    lambda w: {'cmd': 'stop'}),
    (lambda w: w == ['continue'], lambda w: {'cmd': 'continue'}),
    (lambda w: w == ['lift'],     lambda w: {'cmd': 'lift'}),

    # Directional movement
    (lambda w: w == ['forward'],  lambda w: {'cmd': 'move', 'direction': 'forward'}),
    (lambda w: w == ['backward'], lambda w: {'cmd': 'move', 'direction': 'backward'}),
    (lambda w: w == ['left'],     lambda w: {'cmd': 'turn', 'direction': 'left'}),
    (lambda w: w == ['right'],    lambda w: {'cmd': 'turn', 'direction': 'right'}),

    # "lift leave" → place pallet / lower lifter
    (
        lambda w: len(w) == 2 and w[0] == 'lift' and w[1] == 'leave',
        lambda w: {'cmd': 'place'},
    ),

    # "forward stop" → move then stop
    (
        lambda w: len(w) == 2 and w[0] == 'forward' and w[1] == 'stop',
        lambda w: {'cmd': 'stop'},
    ),

    # "start forward" → begin moving forward
    (
        lambda w: len(w) == 2 and w[0] == 'start' and w[1] == 'forward',
        lambda w: {'cmd': 'move', 'direction': 'forward'},
    ),
]


def parse_command(word_sequence: list[str]) -> dict | None:
    """Match a word sequence to a structured command.

    Returns a dict or None if no rule matches.
    """
    for pattern_fn, result_fn in _RULES:
        if pattern_fn(word_sequence):
            return result_fn(word_sequence)
    return None
