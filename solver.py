from json import dump

from interfaces import *
from optimal_transport import *

WORLD_TYPES = ["disk", "sphere", "torus", "klein"]


class Solver:
    def __init__(self, seed, output) -> None:
        self.seed = seed
        self.out = output
        self.paths: dict[str, list[DeliveryPath]] = {}
        self.instances = generate_instances(seed)

    def solve_world(self, map_instance: MapInstance) -> DeliveryPath:
        truck = VEHICLES["truck"]
        jet = VEHICLES["jet"]
        horse = VEHICLES["horse"]
        endpoints = instance.endpoints
        checkpoints = instance.checkpoints
        start = instance.start
    
        direct_endpoint_distances = [geodesic_distance(instance, start, endpoint) for endpoint in endpoints]
        direct_checkpoint_distances = [geodesic_distance(instance, start, checkpoint) for checkpoint in checkpoints]
        midpoint_midpoint_crosses_ocean = [[leg_crosses_ocean(instance, checkpoints[i], checkpoints[j]) for j in range(i)] for i in range(1, N_CHECKPOINTS)]
        midpoint_endpoint_crosses_ocean = [[leg_crosses_ocean(instance, checkpoints[i], endpoints[j]) for j in range(N_ENDPOINTS)] for i in range(N_CHECKPOINTS)]
        start_midpoint_crosses_ocean = [leg_crosses_ocean(instance, start, checkpoint) for checkpoint in checkpoints]
        start_endpoint_crosses_ocean = [leg_crosses_ocean(instance, start, endpoint) for endpoint in endpoints]
    
        def get_candidate_land_vehicle_paths(vehicle: VehicleSpec):
            candidate_midpoint_distances = [[math.inf] * N_CHECKPOINTS for _ in range(N_CHECKPOINTS + 1)]
            candidate_midpoint_traceback = [[-1] * N_CHECKPOINTS for _ in range(N_CHECKPOINTS + 1)]
    
            for i, checkpoint in enumerate(checkpoints):
                if direct_checkpoint_distances[i] <= vehicle.max_leg_m and not start_midpoint_crosses_ocean[i]:
                    candidate_midpoint_distances[1][i] = direct_checkpoint_distances[i]
    
            for n in range(2, N_CHECKPOINTS + 1):
                for i, destination in enumerate(checkpoints):
                    for j, source in enumerate(checkpoints):
                        if i == j or candidate_midpoint_distances[n - 1][j] == math.inf or midpoint_midpoint_crosses_ocean[max(i,j)-1][abs(i-j)-1]:
                            continue
    
                        leg = geodesic_distance(instance, source, destination)
                        new_distance = leg + candidate_midpoint_distances[n - 1][j]
                        if leg <= vehicle.max_leg_m and new_distance <= candidate_midpoint_distances[n][i]:
                            candidate_midpoint_distances[n][i] = new_distance
                            candidate_midpoint_traceback[n][i] = j
    
            candidate_path_distances = [[math.inf] * N_CHECKPOINTS for _ in range(N_CHECKPOINTS + 1)]
            candidate_path_traceback = [[-1] * N_CHECKPOINTS for _ in range(N_CHECKPOINTS + 1)]
    
            for i, endpoint in enumerate(endpoints):
                if direct_endpoint_distances[i] <= vehicle.max_leg_m and not start_endpoint_crosses_ocean[i]:
                    candidate_path_distances[0][i] = direct_endpoint_distances[i]
    
                for n in range(1, N_CHECKPOINTS + 1):
                    for j, midpoint in enumerate(checkpoints):
                        if candidate_midpoint_distances[n][j] == math.inf or midpoint_endpoint_crosses_ocean[j][i]:
                            continue
    
                        leg = geodesic_distance(instance, midpoint, endpoint)
                        new_distance = candidate_midpoint_distances[n][j] + leg
                        if leg <= vehicle.max_leg_m and new_distance <= candidate_path_distances[n][i]:
                            candidate_path_distances[n][i] = new_distance
                            candidate_path_traceback[n][i] = j
    
            return (candidate_path_distances, candidate_path_traceback)
    
        truck_candidate_paths, truck_path_tracebacks = get_candidate_land_vehicle_paths(truck)
        horse_candidate_paths, horse_path_tracebacks = get_candidate_land_vehicle_paths(horse)

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
        self.save()
        for i, world_type in enumerate(WORLD_TYPES):
            self.solve_world(self.instances[world_type])
            self.save()

    def save(self):
        """
        Save the array storing the outputs for the different trees using the paths stored in `self.paths`.
        """
        with open(self.out, mode="w") as f:
            dump({name: [p.get_dict() for p in paths] for name, paths in self.paths.items()}, f)
