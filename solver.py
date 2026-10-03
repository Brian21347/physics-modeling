import multiprocessing
from json import dump

from interfaces import *
from optimal_transport import *

WORLD_TYPES = ["disk", "sphere", "torus", "klein"]
PER_WORLD_TIME_OUTS = [20, 20, 20, 20]


class Solver:
    def __init__(self, seed) -> None:
        self.seed = seed
        self.paths: dict[str, list[DeliveryPath]] = {}
        self.instances = generate_instances(seed)

    def solve_world(self, map_instance: MapInstance) -> DeliveryPath: ...

    def get_base_line(self):
        for world_type in WORLD_TYPES:
            # Constants
            total_tokens = 1000
            world = self.instances[world_type]
            d_index = []
            # Sort by shortest distance
            for i, pt in enumerate(world.endpoints):
                distance = geodesic_distance(world, world.start, pt)  # type: ignore
                d_index.append((distance, i))
            d_index = sorted(d_index)

            # Greedily allocate the horse/truck/jet for small distances
            all_deliveries: list[DeliveryPath] = []
            for distance, i in d_index:
                if distance < 50 * 1000:
                    vehicle = "horse"
                elif distance < 750 * 1000:
                    vehicle = "truck"
                else:
                    vehicle = "jet"
                # evaluate potential delivery method
                delivery = evaluate_delivery(world, i, vehicle, ())
                if total_tokens - delivery.total_tokens > 0:
                    total_tokens -= delivery.total_tokens
                    all_deliveries.append(DeliveryPath(i, vehicle, []))
            self.paths[world_type] = all_deliveries

    def solve(self):
        self.get_base_line()
        for i, world_type in enumerate(WORLD_TYPES):
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
            dump({name: [p.get_dict() for p in paths] for name, paths in self.paths.items()}, f)
