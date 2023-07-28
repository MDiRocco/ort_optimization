"""Simple Vehicles Routing Problem (VRP)."""
import json
import sys

from ortools.constraint_solver import pywrapcp, routing_enums_pb2


class VRP(object):
    """Class for Vehicles Routing Problem."""

    def __init__(self, path_input):
        """Init data for VRP.

        Args:
            path_input: Path for the input files.
        """
        print('INIT')
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
        print(f'Objective: {solution.ObjectiveValue()}')
        max_route_distance = 0
        print(self.input_data)
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
            max_route_distance = max(route_distance, max_route_distance)
        print('Maximum of the route distances: {0}m'.format(max_route_distance))

    @classmethod
    def solve(cls, path):
        """Solve the problem.

        Args:
            path: Path for the input files.

        """
        vrp_object = cls(path)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(vrp_object.input_data['distance_matrix']),
            vrp_object.input_data['num_vehicles'],
            vrp_object.input_data['depot'],
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
            return vrp_object.input_data['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            3000,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name,
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            cls.print_solution(vrp_object, manager, routing, solution)
        else:
            print('No solution found !')
