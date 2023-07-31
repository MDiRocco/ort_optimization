"""Capacited Vehicles Routing Problem (CVRP)."""

import json
import sys

from ortools.constraint_solver import pywrapcp, routing_enums_pb2


class CVRP(object):
    """Class for Capacitated Vehicle Routing Problem."""

    def __init__(self, path_input):
        """Init data for VRP.

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
        total_load = 0
        for vehicle_id in range(self.input_data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_distance = 0
            route_load = 0
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route_load += self.input_data['demands'][node_index]
                plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index,
                    index,
                    vehicle_id,
                )
            plan_output += ' {0} Load({1})\n'.format(
                manager.IndexToNode(index),
                route_load,
            )
            plan_output += 'Distance of the route: {0}m\n'.format(route_distance)
            plan_output += 'Load of the route: {0}\n'.format(route_load)
            print(plan_output)
            total_distance += route_distance
            total_load += route_load
        print('Total distance of all routes: {0}m'.format(total_distance))
        print('Total load of all routes: {0}'.format(total_load))

    @classmethod
    def solve(cls, path):
        """Solve the problem.

        Args:
            path: Path for the input files.

        """
        cvrp_object = cls(path)
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(cvrp_object.input_data['distance_matrix']),
            cvrp_object.input_data['num_vehicles'],
            cvrp_object.input_data['depot'],
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Return the distance between the two nodes.

            Args:
                from_index: start node
                to_index: destination node

            Returns:
                Returns the distance between the two nodes.
            """
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return cvrp_object.input_data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        def demand_callback(from_index):
            """Convert from routing variable Index to demands NodeIndex.

            Args:
                from_index: start node

            Returns:
                Returns the demand of the node.
            """
            from_node = manager.IndexToNode(from_index)
            return cvrp_object.input_data['demands'][from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            cvrp_object.input_data['vehicle_capacities'],  # vehicle maximum capacities
            True,  # noqa: WPS425
            'Capacity',
        )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.FromSeconds(1)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            cvrp_object.print_solution(manager, routing, solution)
