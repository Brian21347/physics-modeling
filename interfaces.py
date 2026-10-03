from dataclasses import dataclass

from optimal_transport import *


@dataclass()
class DeliveryPath:
    endpoint: int
    vehicle: str
    checkpoints: list[int]

    def get_dict(self) -> dict[str, int | str | list]:
        return {"endpoint": self.endpoint, "vehicle": self.vehicle, "checkpoints": self.checkpoints}
