"""
state_machine_node.py
---------------------
ROS2 node that hosts the YASMIN state machine for the AMR warehouse robot.

Responsibilities:
    - Subscribe to /mission, /navigation/status, /alignment_error, /voice_command
    - Publish /navigation/goal, /lifter_level, /robot_state
    - Populate a shared Blackboard with sensor data
    - Build and run the YASMIN StateMachine in a background thread
    - Publish /robot_state on every state transition

State machine topology
----------------------
WAITING_COMMAND
  → mission_received → NAVIGATING_SOURCE
  → stop             → WAITING_COMMAND   (self-loop via "finish" terminal)

NAVIGATING_SOURCE
  → arrived → ALIGNING_TO_PALLET
  → stuck   → WAITING_COMMAND
  → stop    → WAITING_COMMAND

ALIGNING_TO_PALLET
  → aligned → PICKING_DISPATCH
  → failed  → WAITING_COMMAND
  → stop    → WAITING_COMMAND

PICKING_DISPATCH  (reads source_level from blackboard)
  → floor → PICKING_FLOOR
  → rack  → PICKING_RACK
  → truck → PICKING_TRUCK

PICKING_FLOOR / PICKING_RACK / PICKING_TRUCK
  → picked → NAVIGATING_DEST
  → stop   → WAITING_COMMAND

NAVIGATING_DEST
  → arrived → ALIGNING_TO_DEST
  → stuck   → WAITING_COMMAND
  → stop    → WAITING_COMMAND

ALIGNING_TO_DEST
  → aligned → PLACING_PALLET
  → failed  → WAITING_COMMAND
  → stop    → WAITING_COMMAND

PLACING_PALLET
  → placed → WAITING_COMMAND
  → stop   → WAITING_COMMAND
"""

import math
import threading
import logging

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, UInt8
from geometry_msgs.msg import PoseStamped, Point

from yasmin import State, StateMachine, Blackboard

from mission_control.states.waiting_command import WaitingCommand
from mission_control.states.navigating import Navigating
from mission_control.states.aligning import Aligning
from mission_control.states.picking_floor import PickingFloor
from mission_control.states.picking_rack import PickingRack
from mission_control.states.picking_truck import PickingTruck
from mission_control.states.placing_pallet import PlacingPallet

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# PickingDispatcher — lightweight branching state
# ---------------------------------------------------------------------------


class PickingDispatcher(State):
    """Inspect blackboard["current_mission"]["source_level"] and branch.

    Outcomes:
        floor — source_level == 1
        rack  — source_level == 2
        truck — source_level == 3
        stop  — global stop flag or unknown level
    """

    def __init__(self) -> None:
        super().__init__(outcomes=["floor", "rack", "truck", "stop"])

    def execute(self, blackboard: Blackboard) -> str:
        if blackboard.get("stop_flag", False):
            return "stop"

        mission: dict = blackboard.get("current_mission", {})
        source_level: int = mission.get("source_level", 0)

        mapping = {1: "floor", 2: "rack", 3: "truck"}
        outcome = mapping.get(source_level)
        if outcome is None:
            logger.error(
                f"[PickingDispatcher] Unknown source_level={source_level}. "
                "Returning stop."
            )
            return "stop"

        logger.info(
            f"[PickingDispatcher] source_level={source_level} → {outcome}"
        )
        return outcome


# ---------------------------------------------------------------------------
# StateMachineNode
# ---------------------------------------------------------------------------


class StateMachineNode(Node):
    """ROS2 node that owns the shared blackboard and the YASMIN state machine."""

    def __init__(self) -> None:
        super().__init__("state_machine_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("alignment_timeout", 30.0)
        self.declare_parameter("pick_floor_level",  3)
        self.declare_parameter("pick_rack_level",   5)
        self.declare_parameter("pick_truck_level",  3)
        self.declare_parameter("carry_level",       4)
        self.declare_parameter("place_level",       3)
        self.declare_parameter("transport_level",   1)
        self.declare_parameter("rest_level",        0)

        alignment_timeout: float = self.get_parameter("alignment_timeout").value

        # ── Shared blackboard ─────────────────────────────────────────────
        self._blackboard: Blackboard = Blackboard()
        self._blackboard["mission"]           = None
        self._blackboard["current_mission"]   = None
        self._blackboard["nav_status"]        = "IDLE"
        self._blackboard["alignment_error"]   = None
        self._blackboard["stop_flag"]         = False
        self._blackboard["voice_command"]     = None

        # ── QoS ───────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            String,
            "/navigation/status",
            self._nav_status_callback,
            reliable_qos,
        )
        self.create_subscription(
            Point,
            "/alignment_error",
            self._alignment_error_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            "/mission",
            self._mission_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            "/voice_command",
            self._voice_command_callback,
            reliable_qos,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._goal_pub = self.create_publisher(
            PoseStamped, "/navigation/goal", reliable_qos
        )
        self._lifter_pub = self.create_publisher(
            UInt8, "/lifter_level", reliable_qos
        )
        self._state_pub = self.create_publisher(
            String, "/robot_state", reliable_qos
        )

        # ── Build state machine ───────────────────────────────────────────
        self._sm = self._build_state_machine(alignment_timeout)

        # ── Start SM thread ───────────────────────────────────────────────
        # State hooks are already installed during _build_state_machine()
        self._sm_thread = threading.Thread(
            target=self._run_state_machine, daemon=True, name="sm_thread"
        )
        self._sm_thread.start()

        self.get_logger().info("StateMachineNode started.")

    # ── Subscriber callbacks ───────────────────────────────────────────────

    def _nav_status_callback(self, msg: String) -> None:
        """Write navigation status string to the blackboard."""
        self._blackboard["nav_status"] = msg.data.strip().upper()

    def _alignment_error_callback(self, msg: Point) -> None:
        """Write the raw Point message to the blackboard."""
        self._blackboard["alignment_error"] = msg

    def _mission_callback(self, msg: String) -> None:
        """Write the raw mission JSON to the blackboard.

        Only updates if the state machine is idle (mission == None) to avoid
        overwriting an in-progress mission.  A PAUSE/STOP voice command should
        be used to cancel running missions before issuing a new one.
        """
        if self._blackboard.get("mission") is None:
            self._blackboard["mission"] = msg.data
            self.get_logger().info(
                f"[Node] New mission queued: {msg.data[:80]}"
            )
        else:
            self.get_logger().warning(
                "[Node] Mission ignored — previous mission still pending. "
                "Send 'stop' to cancel first."
            )

    def _voice_command_callback(self, msg: String) -> None:
        """Handle recognised voice commands.

        Supported commands (case-insensitive):
            stop / pause  — set stop_flag to True
            resume        — clear stop_flag
        """
        cmd = msg.data.strip().lower()
        self._blackboard["voice_command"] = cmd
        self.get_logger().info(f"[Node] Voice command received: {cmd!r}")

        if cmd in ("stop", "pause"):
            self._blackboard["stop_flag"] = True
            self._blackboard["mission"] = None  # discard pending mission
            self.get_logger().info("[Node] STOP flag raised.")
        elif cmd == "resume":
            self._blackboard["stop_flag"] = False
            self.get_logger().info("[Node] STOP flag cleared (resume).")

    # ── Publisher helpers (injected into states) ───────────────────────────

    def _publish_goal(self, x: float, y: float, theta: float) -> None:
        """Publish a PoseStamped navigation goal."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        half = theta / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self._goal_pub.publish(msg)
        self.get_logger().debug(
            f"[Node] Goal published: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)"
        )

    def _publish_lifter(self, level: int) -> None:
        """Publish a lifter level command (0–7)."""
        msg = UInt8()
        msg.data = max(0, min(7, level))
        self._lifter_pub.publish(msg)
        self.get_logger().debug(f"[Node] Lifter → {msg.data}")

    def _publish_robot_state(self, state_name: str) -> None:
        """Publish the current state machine state name."""
        msg = String()
        msg.data = state_name
        self._state_pub.publish(msg)
        self.get_logger().info(f"[Node] Robot state → {state_name}")

    # ── State machine construction ─────────────────────────────────────────

    def _wrap_with_state_pub(self, state_name: str, state_obj: State) -> State:
        """Return state_obj with its execute() pre-wrapped to publish /robot_state.

        The wrapping mutates the instance method rather than introducing a
        wrapper class, so YASMIN's introspection of outcomes still works on the
        original object.
        """
        original_execute = state_obj.execute
        node_ref = self

        def _hooked_execute(blackboard: Blackboard) -> str:
            node_ref._publish_robot_state(state_name)
            return original_execute(blackboard)

        state_obj.execute = _hooked_execute  # type: ignore[method-assign]
        return state_obj

    def _add(
        self,
        sm: StateMachine,
        name: str,
        state_obj: State,
        transitions: dict[str, str],
    ) -> None:
        """Register a state with /robot_state publishing attached."""
        sm.add_state(name, self._wrap_with_state_pub(name, state_obj), transitions)

    def _build_state_machine(self, alignment_timeout: float) -> StateMachine:
        """Construct and wire the complete YASMIN state machine."""

        sm = StateMachine(outcomes=["finish", "error"])

        # WAITING_COMMAND
        self._add(
            sm, "WAITING_COMMAND",
            WaitingCommand(),
            transitions={
                "mission_received": "NAVIGATING_SOURCE",
                "stop":             "WAITING_COMMAND",
            },
        )

        # NAVIGATING_SOURCE
        self._add(
            sm, "NAVIGATING_SOURCE",
            Navigating(publish_goal_fn=self._publish_goal, phase="to_source"),
            transitions={
                "arrived": "ALIGNING_TO_PALLET",
                "stuck":   "WAITING_COMMAND",
                "stop":    "WAITING_COMMAND",
            },
        )

        # ALIGNING_TO_PALLET
        self._add(
            sm, "ALIGNING_TO_PALLET",
            Aligning(timeout=alignment_timeout),
            transitions={
                "aligned": "PICKING_DISPATCH",
                "failed":  "WAITING_COMMAND",
                "stop":    "WAITING_COMMAND",
            },
        )

        # PICKING_DISPATCH — branches based on source_level
        self._add(
            sm, "PICKING_DISPATCH",
            PickingDispatcher(),
            transitions={
                "floor": "PICKING_FLOOR",
                "rack":  "PICKING_RACK",
                "truck": "PICKING_TRUCK",
                "stop":  "WAITING_COMMAND",
            },
        )

        # PICKING_FLOOR
        self._add(
            sm, "PICKING_FLOOR",
            PickingFloor(publish_lifter_fn=self._publish_lifter),
            transitions={
                "picked": "NAVIGATING_DEST",
                "stop":   "WAITING_COMMAND",
            },
        )

        # PICKING_RACK
        self._add(
            sm, "PICKING_RACK",
            PickingRack(publish_lifter_fn=self._publish_lifter),
            transitions={
                "picked": "NAVIGATING_DEST",
                "stop":   "WAITING_COMMAND",
            },
        )

        # PICKING_TRUCK
        self._add(
            sm, "PICKING_TRUCK",
            PickingTruck(publish_lifter_fn=self._publish_lifter),
            transitions={
                "picked": "NAVIGATING_DEST",
                "stop":   "WAITING_COMMAND",
            },
        )

        # NAVIGATING_DEST
        self._add(
            sm, "NAVIGATING_DEST",
            Navigating(publish_goal_fn=self._publish_goal, phase="to_dest"),
            transitions={
                "arrived": "ALIGNING_TO_DEST",
                "stuck":   "WAITING_COMMAND",
                "stop":    "WAITING_COMMAND",
            },
        )

        # ALIGNING_TO_DEST
        self._add(
            sm, "ALIGNING_TO_DEST",
            Aligning(timeout=alignment_timeout),
            transitions={
                "aligned": "PLACING_PALLET",
                "failed":  "WAITING_COMMAND",
                "stop":    "WAITING_COMMAND",
            },
        )

        # PLACING_PALLET
        self._add(
            sm, "PLACING_PALLET",
            PlacingPallet(publish_lifter_fn=self._publish_lifter),
            transitions={
                "placed": "WAITING_COMMAND",
                "stop":   "WAITING_COMMAND",
            },
        )

        return sm

    # ── SM execution thread ────────────────────────────────────────────────

    def _run_state_machine(self) -> None:
        """Run the state machine loop.  Publishes /robot_state on each tick.

        YASMIN's StateMachine.execute() runs the machine until it reaches a
        terminal outcome ("finish" or "error").  We wrap it in an outer loop
        so the machine restarts automatically — the WAITING_COMMAND state acts
        as the persistent idle point between missions.
        """
        self.get_logger().info("[SM Thread] Starting state machine loop.")

        while rclpy.ok():
            try:
                self.get_logger().info("[SM Thread] Executing state machine.")
                outcome = self._sm.execute(self._blackboard)
                self.get_logger().info(
                    f"[SM Thread] State machine finished with outcome: {outcome!r}"
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"[SM Thread] Unhandled exception in state machine: {exc}",
                    exc_info=True,
                )
                self._publish_robot_state("ERROR")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> None:
    rclpy.init()
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down StateMachineNode.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
