def check_all_ids_are_unique(params):
    ids = [param["id"] for param in params]
    all_unique = len(ids) == len(set(ids))
    if not all_unique:
        not_unique_ids = [id for id in ids if ids.count(id) > 1]
        raise ValueError(f"Agent ids are not unique: {not_unique_ids}")
    return all_unique


def create_step_name(step: int, verision: int, robot_sensors: str) -> str:
    assert robot_sensors in ["front", "front-back", "sides", "front-back-6"], "Invalid robot_sensors. Must be one of: front, front-back, sides, front-back-6."  # type: ignore
    return f"step-{step}-v{verision}-{robot_sensors}"
