import multiprocessing
from enum import Enum
from json import dump

from interfaces import *
from optimal_transport import *

WORLD_TYPES = ["disk", "sphere", "torus", "klein"]
PER_WORLD_TIME_OUTS = [30, 30, 30, 30]


class Solver:
    def __init__(self, seed) -> None:
        self.seed = seed
        self.paths: dict[str, DeliveryPath] = {}
        self.instances = generate_instances(seed)

    def solve_world(self, map_instance: MapInstance) -> DeliveryPath: ...

    def default_path(self, map_instance: MapInstance) -> DeliveryPath: ...

    def solve(self):
        for i, world_type in enumerate(WORLD_TYPES):
            default = self.default_path(self.instances[world_type])
            self.paths[world_type] = default
            process = multiprocessing.Process(
                target=self.solve_world, args=(self.instances[world_type],)
            )
            process.start()
            process.join(timeout=PER_WORLD_TIME_OUTS[i])

            if process.is_alive():
                process.terminate()
                process.join()

    def save(self, output: str):
        """
        Save the array storing the outputs for the different trees using the paths stored in `self.paths`.
        """
        with open(output, mode="w") as f:
            dump(self.paths, f)
