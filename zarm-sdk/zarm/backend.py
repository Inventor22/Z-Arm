"""The backend seam: one method per vendor C function the SDK uses.

Backends take and return raw vendor values (degrees, millimetres, integer
result codes). Validation, typing, error mapping and waiting live in
:class:`zarm.ZArm`, so a backend stays a thin, mechanical translation layer.
``zarm.native.NativeBackend`` calls Hitbot's DLL; ``zarm.fake.FakeBackend``
simulates it.
"""

from __future__ import annotations

import abc
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .errors import LibraryError, OwnershipError


@dataclass(frozen=True)
class ScaraParam:
    """Output of ``robot_get_scara_param``.

    ``initial_finish`` is the controller's "initialized" flag. (The old vendor
    header comment says 0 means success; the DLL's own code treats 1 as
    initialized, see docs/native-api.md.)
    """

    x_mm: float
    y_mm: float
    z_mm: float
    angle1_deg: float
    angle2_deg: float
    r_deg: float
    communicate_success: bool
    initial_finish: bool
    move_flag: bool


@dataclass(frozen=True)
class EncoderParam:
    """Output of ``robot_get_encoder_param``."""

    x_mm: float
    y_mm: float
    z_mm: float
    angle1_deg: float
    angle2_deg: float
    r_deg: float


class Backend(abc.ABC):
    """Vendor-shaped operations plus lifetime bookkeeping.

    Every ZArm using a backend holds a client lease from before it starts the
    server until it closes, and owns at most one card. The vendor server is
    shared by all clients, so it is only shut down when the last lease goes.
    """

    def __init__(self) -> None:
        self._owners: Dict[int, object] = {}
        self._clients: List[object] = []
        self._close_when_unused = False
        self._owners_lock = threading.Lock()

    def acquire_client(self, client: object) -> None:
        with self._owners_lock:
            if not any(existing is client for existing in self._clients):
                self._clients.append(client)

    def release_client(self, client: object, *, stop_server: bool = False) -> bool:
        """Drop ``client`` and its cards. With ``stop_server``, shut the server down if no client remains.

        Returns True if the server was shut down. The check and the shutdown are
        atomic with respect to :meth:`acquire_client` and :meth:`claim`.
        """
        with self._owners_lock:
            self._clients = [existing for existing in self._clients if existing is not client]
            for card in [card for card, owner in self._owners.items() if owner is client]:
                del self._owners[card]
            shut_down = False
            if stop_server and not self._clients:
                self.close_server()
                shut_down = True
            close_now = self._close_when_unused and not self._clients
        if close_now:
            self.close_when_unused()
        return shut_down

    def close_when_unused(self) -> None:
        """Close now if no client holds a lease, otherwise when the last one is released."""
        with self._owners_lock:
            if self._clients:
                self._close_when_unused = True
                return
            self._close_when_unused = False
        try:
            self.close()
        except LibraryError:
            # A client arrived in the meantime; close when it leaves.
            with self._owners_lock:
                self._close_when_unused = True

    def has_clients(self) -> bool:
        with self._owners_lock:
            return bool(self._clients)

    def claim(self, card: int, owner: object) -> None:
        with self._owners_lock:
            current = self._owners.get(card)
            if current is not None and current is not owner:
                raise OwnershipError(f"card {card} is already owned by {current!r}")
            self._owners[card] = owner

    def release(self, card: int, owner: object) -> None:
        with self._owners_lock:
            if self._owners.get(card) is owner:
                del self._owners[card]

    def owner_of(self, card: int) -> Optional[object]:
        with self._owners_lock:
            return self._owners.get(card)

    def owned_cards(self) -> Tuple[int, ...]:
        with self._owners_lock:
            return tuple(sorted(self._owners))

    def close(self) -> None:
        """Release backend resources. The default does nothing."""

    # Server and discovery

    @abc.abstractmethod
    def start_server(self) -> int:
        """``net_port_initial``: 1 on success, -1 on failure."""

    @abc.abstractmethod
    def close_server(self) -> None:
        """``close_server``: ask the vendor server process to exit."""

    @abc.abstractmethod
    def card_connect_status(self, card: int) -> int:
        """``card_number_is_connect``: 1 connected, 0 not connected, 2 card out of range."""

    # Setup

    @abc.abstractmethod
    def initial(self, card: int, generation: int, z_travel_mm: int) -> int:
        """``robot_initial``."""

    @abc.abstractmethod
    def unlock_position(self, card: int) -> int:
        """``robot_unlock_position``: 1 on success, 0 when not connected."""

    @abc.abstractmethod
    def set_arm_length(self, card: int, l1_mm: float, l2_mm: float) -> None:
        """``robot_set_arm_length``."""

    # State

    @abc.abstractmethod
    def get_scara_param(self, card: int) -> ScaraParam:
        """``robot_get_scara_param``."""

    @abc.abstractmethod
    def get_encoder_param(self, card: int) -> EncoderParam:
        """``robot_get_encoder_param``."""

    @abc.abstractmethod
    def get_joint_state(self, card: int, joint: int) -> int:
        """``robot_get_joint_state``."""

    @abc.abstractmethod
    def is_collision(self, card: int) -> bool:
        """``robot_is_collision``."""

    @abc.abstractmethod
    def is_stop(self, card: int) -> bool:
        """``robot_is_stop``: True when the arm is not moving."""

    # Motion

    @abc.abstractmethod
    def movej_angle(
        self, card: int, angle1_deg: float, angle2_deg: float, z_mm: float, r_deg: float, speed: float, roughly: float
    ) -> int:
        """``robot_movej_angle``: joint-space move to absolute joint values."""

    @abc.abstractmethod
    def movej_xyz(
        self, card: int, x_mm: float, y_mm: float, z_mm: float, r_deg: float, speed: float, roughly: float
    ) -> int:
        """``robot_movej_xyz``: joint-interpolated move to a Cartesian target."""

    @abc.abstractmethod
    def movej_xyz_lr(
        self,
        card: int,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        r_deg: float,
        speed: float,
        roughly: float,
        lr: int,
    ) -> int:
        """``robot_movej_xyz_lr``: as movej_xyz with an explicit hand configuration."""

    @abc.abstractmethod
    def movel_xyz(self, card: int, x_mm: float, y_mm: float, z_mm: float, r_deg: float, speed_mm_s: float) -> int:
        """``robot_movel_xyz``: straight-line Cartesian move."""

    @abc.abstractmethod
    def change_attitude(self, card: int, speed: float) -> int:
        """``robot_change_attitude``: switch to the other hand (elbow) configuration."""

    @abc.abstractmethod
    def joint_home(self, card: int, joint: int) -> int:
        """``robot_joint_home``. Clears initialization on entry (even when refused), then moves the joint."""

    @abc.abstractmethod
    def stop_move(self, card: int) -> None:
        """``robot_stop_move``. Must be callable while another call is in progress."""

    @abc.abstractmethod
    def emergency_stop(self, card: int) -> None:
        """``robot_emergency_stop``. Must be callable while another call is in progress."""

    @abc.abstractmethod
    def pause_move(self, card: int) -> int:
        """``robot_pause_move``: 1 on success."""

    @abc.abstractmethod
    def resume_move(self, card: int) -> int:
        """``robot_resume_move``: 1 on success."""

    # Modes and I/O

    @abc.abstractmethod
    def set_drag_teach(self, card: int, enable: bool) -> bool:
        """``robot_set_drag_teach``."""

    @abc.abstractmethod
    def get_drag_teach(self, card: int) -> bool:
        """``robot_get_drag_teach``."""

    @abc.abstractmethod
    def set_cooperation_fun_state(self, card: int, enable: bool) -> bool:
        """``robot_set_cooperation_fun_state``."""

    @abc.abstractmethod
    def get_cooperation_fun_state(self, card: int) -> bool:
        """``robot_get_cooperation_fun_state``."""

    @abc.abstractmethod
    def get_digital_io_in(self, card: int, io: int) -> int:
        """``robot_get_digital_io_in``: 0/1 level, -1 invalid io, 3 not connected."""

    @abc.abstractmethod
    def get_digital_io_out(self, card: int, io: int) -> int:
        """``robot_get_digital_io_out``: 0/1 level, -1 invalid io, 3 not connected."""

    @abc.abstractmethod
    def set_digital_io_out(self, card: int, io: int, value: bool) -> bool:
        """``robot_set_digital_io_out``."""
