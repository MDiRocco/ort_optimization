"""Traveling Salesperson Problem."""

import json
import sys

from ortools.constraint_solver import pywrapcp, routing_enums_pb2


class TSP(object):
    """Class for Traveling Salesperson Problem."""

    def __init__(self, path_input):
        """Init data for TSP.

        Args:
            path_input: Path for the input files.
        """
        self.path_input = path_input
        self.input_data = {}
        self.create_data_model()

    def create_data_model(self):
        """Store the data for the problem."""
        # with open("sample.json", "w") as outfile:
        #     json.dump(self.input_data, outfile)
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
        print('Objective: {0} miles'.format(solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route for vehicle 0:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {0} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {0}\n'.format(manager.IndexToNode(index))
        print(plan_output)
        plan_output += 'Route distance: {0}miles\n'.format(route_distance)

    @classmethod
    def solve(cls, path):
        """Solve the problem.

        Args:
            path: Path for the input files.

        """
        tsp_object = cls(path)
        # print(classe.input_data.keys())

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(tsp_object.input_data['distance_matrix']),
            tsp_object.input_data['num_vehicles'],
            tsp_object.input_data['depot'],
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

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
            return tsp_object.input_data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            tsp_object.print_solution(manager, routing, solution)
