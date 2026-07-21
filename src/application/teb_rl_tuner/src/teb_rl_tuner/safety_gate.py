"""Fail-closed gate for T03 simulation parameter writes."""

from dataclasses import dataclass


EXPECTED_TEB_NAMESPACE = "/move_base/TebLocalPlannerROS"


class SimulationGateError(RuntimeError):
    """Raised when a parameter write is not proven to target the T02 simulator."""


@dataclass(frozen=True)
class SimulationWriteContext:
    """Evidence required before the T03 client may issue a write."""

    explicit_simulation: bool
    use_sim_time: bool
    simulation_marker: bool
    teb_namespace: str


def require_t02_simulation(context: SimulationWriteContext) -> None:
    """Reject writes unless all independent simulation guards are explicit."""

    failures = []
    if context.explicit_simulation is not True:
        failures.append("explicit simulation flag is not true")
    if context.use_sim_time is not True:
        failures.append("/use_sim_time is not true")
    if context.simulation_marker is not True:
        failures.append("/m2_gazebo/simulation_only is not true")
    if context.teb_namespace != EXPECTED_TEB_NAMESPACE:
        failures.append("TEB namespace is not the T02 namespace")
    if failures:
        raise SimulationGateError("parameter write denied: " + "; ".join(failures))
