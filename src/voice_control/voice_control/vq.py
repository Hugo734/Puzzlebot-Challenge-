"""Vector Quantisation (VQ) codebook for voice recognition.

Each word in the vocabulary gets its own VQCodebook trained on LSF features
extracted from multiple recordings of that word.  Recognition is performed by
finding the codebook that yields the minimum average distortion for a new
utterance.
"""

from __future__ import annotations

import logging
import os
import pickle
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


class VQCodebook:
    """VQ codebook backed by sklearn KMeans (LBG approximation).

    Args:
        n_clusters: Number of codewords (default 32).
    """

    def __init__(self, n_clusters: int = 32) -> None:
        self._n_clusters = n_clusters
        self._kmeans: object | None = None  # sklearn KMeans instance

    # ── Training ──────────────────────────────────────────────────────

    def train(self, features: np.ndarray) -> None:
        """Fit the codebook to a set of feature vectors.

        Args:
            features: 2-D array of shape ``(n_samples, n_features)``.
        """
        from sklearn.cluster import KMeans  # noqa: PLC0415

        n_clusters = min(self._n_clusters, features.shape[0])
        self._kmeans = KMeans(
            n_clusters=n_clusters,
            n_init=10,
            random_state=42,
        )
        self._kmeans.fit(features)
        logger.info(
            'VQCodebook trained: %d clusters, %d samples, %d features',
            n_clusters,
            features.shape[0],
            features.shape[1],
        )

    # ── Recognition ───────────────────────────────────────────────────

    def distortion(self, features: np.ndarray) -> float:
        """Mean Euclidean distance from feature vectors to nearest centroid.

        Args:
            features: 2-D array of shape ``(n_frames, n_features)``.

        Returns:
            Mean distortion scalar.  Returns ``float('inf')`` if the codebook
            has not been trained or ``features`` is empty.
        """
        if self._kmeans is None or features.shape[0] == 0:
            return float('inf')

        centroids: np.ndarray = self._kmeans.cluster_centers_
        # Compute Euclidean distance from each frame to every centroid
        diffs = features[:, np.newaxis, :] - centroids[np.newaxis, :, :]  # (F, K, D)
        distances = np.sqrt(np.sum(diffs ** 2, axis=2))  # (F, K)
        min_distances = np.min(distances, axis=1)  # (F,)
        return float(np.mean(min_distances))

    # ── Persistence ───────────────────────────────────────────────────

    def save(self, path: str) -> None:
        """Serialise codebook to a pickle file.

        Args:
            path: Destination file path (created if absent).
        """
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'wb') as fh:
            pickle.dump({'n_clusters': self._n_clusters, 'kmeans': self._kmeans}, fh)
        logger.info('VQCodebook saved to %s', path)

    def load(self, path: str) -> None:
        """Deserialise codebook from a pickle file.

        Args:
            path: Source file path.

        Raises:
            FileNotFoundError: If the file does not exist.
        """
        with open(path, 'rb') as fh:
            data = pickle.load(fh)  # noqa: S301
        self._n_clusters = data['n_clusters']
        self._kmeans = data['kmeans']
        logger.info('VQCodebook loaded from %s', path)


# ── Batch training helper ─────────────────────────────────────────────────────

def train_all_codebooks(
    training_data: dict[str, np.ndarray],
    n_clusters: int = 32,
    codebook_dir: str = 'codebooks',
) -> None:
    """Train and save one VQCodebook per word in ``training_data``.

    Args:
        training_data: Mapping of ``{word: feature_array}`` where
                       ``feature_array`` has shape ``(n_samples, n_features)``.
        n_clusters:    Number of VQ codewords per codebook.
        codebook_dir:  Directory where ``{word}.pkl`` files are saved.
    """
    os.makedirs(codebook_dir, exist_ok=True)

    for word, features in training_data.items():
        logger.info('Training codebook for word "%s" (%d frames) ...', word, features.shape[0])
        codebook = VQCodebook(n_clusters=n_clusters)
        codebook.train(features)
        save_path = os.path.join(codebook_dir, f'{word}.pkl')
        codebook.save(save_path)

    logger.info('All codebooks saved to %s/', codebook_dir)
