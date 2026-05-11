"""Grammar-based command parser for the AMR voice interface.

Takes a sequence of recognised words and maps them to structured mission
commands that the state machine can consume.
"""

from __future__ import annotations

VOCABULARY: list[str] = [
    'start', 'stop', 'pause', 'next',
    've', 'rack', 'camion', 'roller',
    'nivel', 'uno', 'dos',
    'recoge', 'deja',
]

# ── Grammar rules ─────────────────────────────────────────────────────────────
# Each rule is a tuple:  (pattern_fn, result_fn)
# pattern_fn(words) -> bool   — True when this rule matches
# result_fn(words)  -> dict   — builds the structured command

_RULES: list[tuple] = [
    # Simple control commands
    (
        lambda w: w == ['start'],
        lambda w: {'cmd': 'start'},
    ),
    (
        lambda w: w == ['stop'],
        lambda w: {'cmd': 'stop'},
    ),
    (
        lambda w: w == ['pause'],
        lambda w: {'cmd': 'pause'},
    ),
    (
        lambda w: w == ['next'],
        lambda w: {'cmd': 'next'},
    ),

    # "ve rack nivel uno recoge" → pick from rack level 1
    (
        lambda w: (
            len(w) == 5
            and w[0] == 've'
            and w[1] == 'rack'
            and w[2] == 'nivel'
            and w[3] in ('uno', 'dos')
            and w[4] == 'recoge'
        ),
        lambda w: {
            'cmd': 'mission',
            'source': 'rack_1' if w[3] == 'uno' else 'rack_2',
            'action': 'pick',
        },
    ),

    # "ve camion deja" → place pallet at truck
    (
        lambda w: len(w) == 3 and w[0] == 've' and w[1] == 'camion' and w[2] == 'deja',
        lambda w: {'cmd': 'mission', 'dest': 'truck', 'action': 'place'},
    ),

    # "ve roller recoge" → pick from roller
    (
        lambda w: len(w) == 3 and w[0] == 've' and w[1] == 'roller' and w[2] == 'recoge',
        lambda w: {'cmd': 'mission', 'zone': 'roller', 'action': 'pick'},
    ),

    # "ve roller deja" → place at roller
    (
        lambda w: len(w) == 3 and w[0] == 've' and w[1] == 'roller' and w[2] == 'deja',
        lambda w: {'cmd': 'mission', 'zone': 'roller', 'action': 'place'},
    ),
]


def parse_command(word_sequence: list[str]) -> dict | None:
    """Match a word sequence to a structured command using grammar rules.

    Args:
        word_sequence: Ordered list of recognised words (e.g. ``['ve', 'rack',
                       'nivel', 'uno', 'recoge']``).

    Returns:
        A dict describing the command, or ``None`` if no rule matches.

    Examples:
        >>> parse_command(['start'])
        {'cmd': 'start'}
        >>> parse_command(['ve', 'rack', 'nivel', 'dos', 'recoge'])
        {'cmd': 'mission', 'source': 'rack_2', 'action': 'pick'}
        >>> parse_command(['ve', 'camion', 'deja'])
        {'cmd': 'mission', 'dest': 'truck', 'action': 'place'}
    """
    for pattern_fn, result_fn in _RULES:
        if pattern_fn(word_sequence):
            return result_fn(word_sequence)

    return None
