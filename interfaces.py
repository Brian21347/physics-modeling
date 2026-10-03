from dataclasses import dataclass

from optimal_transport import *


@dataclass()
class DeliveryPath:
    endpoint: int
    vehicle: str
    checkpoints: list[int]

    #     {
    #   "disk":  [{"endpoint": 0, "vehicle": "truck", "checkpoints": []}],
    #   "sphere": [...],
    #   "torus":  [...],
    #   "klein":  [...]
    # }
