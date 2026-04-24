from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Mapping, Sequence


CRX_CONFIG_STRIDE = 6


class CrxWristConfig(IntEnum):
    Unknown = -1
    NonFlip = 0
    Flip = 1


class CrxElbowConfig(IntEnum):
    Unknown = -1
    Up = 0
    Down = 1


class CrxShoulderConfig(IntEnum):
    Unknown = -1
    Top = 0
    Bottom = 1


_FIXTURE_WRIST = {
    "N": CrxWristConfig.NonFlip,
    "F": CrxWristConfig.Flip,
}

_FIXTURE_ELBOW = {
    "U": CrxElbowConfig.Up,
    "D": CrxElbowConfig.Down,
}

_FIXTURE_SHOULDER = {
    "T": CrxShoulderConfig.Top,
    "B": CrxShoulderConfig.Bottom,
}

_FANUC_WRIST = {
    CrxWristConfig.NonFlip: "N",
    CrxWristConfig.Flip: "F",
    CrxWristConfig.Unknown: "?",
}

_FANUC_ELBOW = {
    CrxElbowConfig.Up: "U",
    CrxElbowConfig.Down: "D",
    CrxElbowConfig.Unknown: "?",
}

_FANUC_SHOULDER = {
    CrxShoulderConfig.Top: "T",
    CrxShoulderConfig.Bottom: "B",
    CrxShoulderConfig.Unknown: "?",
}


def _wrist_from_int(value: int) -> CrxWristConfig:
    try:
        return CrxWristConfig(int(value))
    except ValueError:
        return CrxWristConfig.Unknown


def _elbow_from_int(value: int) -> CrxElbowConfig:
    try:
        return CrxElbowConfig(int(value))
    except ValueError:
        return CrxElbowConfig.Unknown


def _shoulder_from_int(value: int) -> CrxShoulderConfig:
    try:
        return CrxShoulderConfig(int(value))
    except ValueError:
        return CrxShoulderConfig.Unknown


@dataclass(frozen=True)
class CrxConfiguration:
    wrist: CrxWristConfig = CrxWristConfig.Unknown
    elbow: CrxElbowConfig = CrxElbowConfig.Unknown
    shoulder: CrxShoulderConfig = CrxShoulderConfig.Unknown

    @classmethod
    def from_fixture(cls, config: Mapping[str, str]) -> CrxConfiguration:
        # Fixture key FB is FANUC wrist flip/non-flip, not shoulder front/back.
        return cls(
            wrist=_FIXTURE_WRIST.get(config.get("FB", ""), CrxWristConfig.Unknown),
            elbow=_FIXTURE_ELBOW.get(config.get("UD", ""), CrxElbowConfig.Unknown),
            shoulder=_FIXTURE_SHOULDER.get(
                config.get("TB", ""), CrxShoulderConfig.Unknown
            ),
        )

    @classmethod
    def from_robodk_vector(cls, config: Sequence[int]) -> CrxConfiguration:
        wrist = _wrist_from_int(config[2]) if len(config) > 2 else CrxWristConfig.Unknown
        elbow = _elbow_from_int(config[1]) if len(config) > 1 else CrxElbowConfig.Unknown
        shoulder = (
            _shoulder_from_int(config[0])
            if len(config) > 0
            else CrxShoulderConfig.Unknown
        )
        return cls(wrist=wrist, elbow=elbow, shoulder=shoulder)

    def is_known(self) -> bool:
        return (
            self.wrist != CrxWristConfig.Unknown
            and self.elbow != CrxElbowConfig.Unknown
            and self.shoulder != CrxShoulderConfig.Unknown
        )

    def to_fanuc_string(self) -> str:
        return (
            _FANUC_WRIST[self.wrist]
            + _FANUC_ELBOW[self.elbow]
            + _FANUC_SHOULDER[self.shoulder]
        )

    def to_robodk_vector(self) -> list[int]:
        return [
            int(self.shoulder),
            int(self.elbow),
            int(self.wrist),
            0,
            0,
            0,
        ]
