"""Vehicle Routing with Pickup Delivery Problem (PDP)."""
import json
import sys

from ortools.constraint_solver import pywrapcp, routing_enums_pb2


class PDP(object):
    """Class for Pickup Delivery Problem."""

    def __init__(self, path_input):
        """Init data for PDP.

        Args:
            path_input: Path for the input files.
        """
        self.path_input = path_input
        self.input_data = {}
        self.create_data_model()

    def create_data_model(self):
        """Store the data for the problem."""
        try:
            with open(self.path_input) as json_file:
                self.input_data = json.load(json_file)
        except json.decoder.JSONDecodeError as er:
            print('JSON VALIDATION FAILED')
            print(er)
            sys.exit(1)

    def print_solution(self, manager, routing, solution):
        """Print solution on console.

        Args:
            manager: Manager for any NodeIndex <-> variable index conversion.
            routing: Routing Model
            solution: Solves the current routing model with the given parameters
        """
        print(f'Objective: {solution.ObjectiveValue()}')
        total_distance = 0
        for vehicle_id in range(self.input_data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_distance = 0
            while not routing.IsEnd(index):
                plan_output += ' {0} -> '.format(manager.IndexToNode(index))
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id,
                )
            plan_output += '{0}\n'.format(manager.IndexToNode(index))
            plan_output += 'Distance of the route: {0}m\n'.format(route_distance)
            print(plan_output)
            total_distance += route_distance
        print('Total Distance of all routes: {0}m'.format(total_distance))

    @classmethod
    def solve(cls, path):
        """Solve the problem.

        Args:
            path: Path for the input files.

        """
        pdp_object = cls(path)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(pdp_object.input_data['distance_matrix']),
            pdp_object.input_data['num_vehicles'],
            pdp_object.input_data['depot'],
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Define cost of each arc.
        def distance_callback(from_index, to_index):
            """Convert from routing variable Index to distance matrix NodeIndex.

            Args:
                from_index: start node
                to_index: destination node

            Returns:
                Returns the distance between the two nodes.
            """
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return pdp_object.input_data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            pdp_object.input_data['travel distance'],  # vehicle maximum travel distance
            True,  # noqa:WPS425 start cumul to zero
            dimension_name,
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Define Transportation Requests.
        for request in pdp_object.input_data['pickups_deliveries']:
            pickup_index = manager.NodeToIndex(request[0])
            delivery_index = manager.NodeToIndex(request[1])
            routing.AddPickupAndDelivery(pickup_index, delivery_index)
            routing.solver().Add(
                routing.VehicleVar(pickup_index) == routing.VehicleVar(delivery_index),
            )
            routing.solver().Add(
                distance_dimension.CumulVar(pickup_index) <=
                distance_dimension.CumulVar(delivery_index),
            )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            pdp_object.print_solution(manager, routing, solution)