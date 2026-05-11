"""ROS2 node for LPC+VQ voice recognition.

Publications
------------
/voice_command  (std_msgs/String)
    Single recognised word, published after each utterance.

/mission  (std_msgs/String)
    JSON-serialised mission dict, published when the word buffer produces a
    valid grammar match via command_parser.

Subscriptions (mock_mode only)
-------------------------------
/mock_voice  (std_msgs/String)
    Inject words directly (e.g. from CLI or another node) for testing without
    a microphone.

Parameters
----------
See config/voice_params.yaml for the full list.
"""

from __future__ import annotations

import json
import logging
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from voice_control.command_parser import VOCABULARY, parse_command
from voice_control.recognizer import VoiceRecognizer

logger = logging.getLogger(__name__)


class VoiceNode(Node):
    """Bridges microphone audio (or mock injection) to ROS2 voice commands."""

    def __init__(self) -> None:
        super().__init__('voice_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('codebook_dir', '')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('lpc_order', 12)
        self.declare_parameter('vq_clusters', 32)
        self.declare_parameter('frame_ms', 25)
        self.declare_parameter('hop_ms', 10)
        self.declare_parameter('energy_threshold_factor', 3.0)
        self.declare_parameter('min_speech_ms', 200)
        self.declare_parameter('silence_ms', 500)
        self.declare_parameter('word_buffer_size', 5)
        self.declare_parameter('rejection_threshold', 2.0)
        self.declare_parameter('mock_mode', False)

        p = self.get_parameter
        self._sample_rate: int = p('sample_rate').get_parameter_value().integer_value
        self._lpc_order: int = p('lpc_order').get_parameter_value().integer_value
        self._frame_ms: int = p('frame_ms').get_parameter_value().integer_value
        self._hop_ms: int = p('hop_ms').get_parameter_value().integer_value
        self._word_buffer_size: int = p('word_buffer_size').get_parameter_value().integer_value
        self._rejection_threshold: float = p('rejection_threshold').get_parameter_value().double_value
        self._mock_mode: bool = p('mock_mode').get_parameter_value().bool_value

        codebook_dir: str = p('codebook_dir').get_parameter_value().string_value
        if not codebook_dir:
            # Default: <package_share>/codebooks  (falls back to install-time path)
            codebook_dir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'codebooks',
            )
        self._codebook_dir = codebook_dir

        # ── State ─────────────────────────────────────────────────────
        self._word_buffer: list[str] = []

        # ── ROS interfaces ────────────────────────────────────────────
        self._voice_pub = self.create_publisher(String, '/voice_command', 10)
        self._mission_pub = self.create_publisher(String, '/mission', 10)

        # ── Recognizer ────────────────────────────────────────────────
        self._recognizer = VoiceRecognizer(
            codebook_dir=self._codebook_dir,
            vocabulary=VOCABULARY,
            lpc_order=self._lpc_order,
            frame_ms=self._frame_ms,
            hop_ms=self._hop_ms,
            sample_rate=self._sample_rate,
            rejection_threshold=self._rejection_threshold,
        )
        self._recognizer.load_codebooks()

        if not self._recognizer.is_ready:
            self.get_logger().warn(
                'Not all codebooks loaded.  Train first with: '
                'ros2 run voice_control train'
            )

        # ── Mode ──────────────────────────────────────────────────────
        if self._mock_mode:
            self._setup_mock_mode()
        else:
            self._setup_real_mode()

        self.get_logger().info(
            'VoiceNode ready — mock_mode=%s, codebook_dir=%s',
            self._mock_mode,
            self._codebook_dir,
        )

    # ── Mock mode ─────────────────────────────────────────────────────

    def _setup_mock_mode(self) -> None:
        """Subscribe to /mock_voice for injecting words without a microphone."""
        self._mock_sub = self.create_subscription(
            String,
            '/mock_voice',
            self._on_mock_voice,
            10,
        )
        self.get_logger().info(
            'VoiceNode in mock mode — publish words to /mock_voice to test'
        )

    def _on_mock_voice(self, msg: String) -> None:
        """Treat the incoming string as a recognised word."""
        word = msg.data.strip().lower()
        if word:
            self.get_logger().info('Mock voice word received: "%s"', word)
            self._handle_word(word)

    # ── Real microphone mode ───────────────────────────────────────────

    def _setup_real_mode(self) -> None:
        """Launch the audio capture and recognition pipeline in a background thread."""
        self._stop_event = threading.Event()
        self._audio_thread = threading.Thread(
            target=self._audio_loop,
            name='voice_audio_loop',
            daemon=True,
        )
        self._audio_thread.start()
        self.get_logger().info('VoiceNode audio thread started')

    def _audio_loop(self) -> None:
        """Blocking loop: capture audio, endpoint utterances, recognise words."""
        try:
            from voice_control.endpoint import VoiceEndpointer  # noqa: PLC0415
            from voice_control.record import AudioRecorder  # noqa: PLC0415
        except ImportError as exc:
            self.get_logger().error('Failed to import audio modules: %s', exc)
            return

        energy_factor: float = (
            self.get_parameter('energy_threshold_factor').get_parameter_value().double_value
        )
        min_speech_ms: int = (
            self.get_parameter('min_speech_ms').get_parameter_value().integer_value
        )
        silence_ms: int = (
            self.get_parameter('silence_ms').get_parameter_value().integer_value
        )

        recorder = AudioRecorder(
            sample_rate=self._sample_rate,
            chunk_size=512,
            channels=1,
        )
        endpointer = VoiceEndpointer(
            sample_rate=self._sample_rate,
            energy_threshold_factor=energy_factor,
            min_speech_ms=min_speech_ms,
            silence_ms=silence_ms,
        )

        try:
            recorder.start()
            self.get_logger().info('Microphone stream open — listening ...')

            while not self._stop_event.is_set():
                chunk = recorder.read_chunk()
                utterance = endpointer.process(chunk)
                if utterance is not None:
                    self._process_utterance(utterance)

        except RuntimeError as exc:
            self.get_logger().error('AudioRecorder error: %s', exc)
        finally:
            recorder.stop()

    def _process_utterance(self, utterance: np.ndarray) -> None:
        """Run the recognizer on a detected utterance and handle the result."""
        word = self._recognizer.recognize(utterance)
        if word is not None:
            self.get_logger().info('Recognised: "%s"', word)
            self._handle_word(word)

    # ── Shared word handling ───────────────────────────────────────────

    def _handle_word(self, word: str) -> None:
        """Publish the word on /voice_command and attempt grammar matching."""
        # Publish raw word
        msg = String()
        msg.data = word
        self._voice_pub.publish(msg)

        # Buffer and try to parse a command
        self._word_buffer.append(word)
        if len(self._word_buffer) > self._word_buffer_size:
            self._word_buffer.pop(0)

        command = parse_command(self._word_buffer)
        if command is not None:
            self.get_logger().info('Command parsed: %s', command)
            mission_msg = String()
            mission_msg.data = json.dumps(command)
            self._mission_pub.publish(mission_msg)
            self._word_buffer.clear()

    # ── Lifecycle ─────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if hasattr(self, '_stop_event'):
            self._stop_event.set()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
