"""Vehicle Routing Problems with Time Windows and Depot Constraints."""

import json
import sys

from ortools.constraint_solver import pywrapcp, routing_enums_pb2


class TWDCP(object):
    """Class for Vehicle Routing Problems with Time Windows adn Depot Constraints."""

    def __init__(self, path_input):
        """Init data for TWDCP.

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
        time_dimension = routing.GetDimensionOrDie('Time')
        total_time = 0
        for vehicle_id in range(self.input_data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                plan_output += '{0} Time({1},{2}) -> '.format(
                    manager.IndexToNode(index), solution.Min(time_var), solution.Max(time_var),
                )
                index = solution.Value(routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2})\n'.format(
                manager.IndexToNode(index), solution.Min(time_var), solution.Max(time_var),
            )
            plan_output += 'Time of the route: {0}min\n'.format(solution.Min(time_var))
            print(plan_output)
            total_time += solution.Min(time_var)
        print('Total time of all routes: {0}min'.format(total_time))

    @classmethod
    def solve(cls, path):
        """Solve the VRP with time windows.

        Args:
            path: Path for the input files.

        """
        twdcp_object = cls(path)

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(twdcp_object.input_data['time_matrix']),
            twdcp_object.input_data['num_vehicles'],
            twdcp_object.input_data['depot'],
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def time_callback(from_index, to_index):
            """Convert from routing variable Index to time matrix NodeIndex.

            Args:
                from_index: start node
                to_index: destination node

            Returns:
                Return the travel time between the two nodes.
            """
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return twdcp_object.input_data['time_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(time_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Time Windows constraint.
        dimension_name = 'Time'
        twdcp_object.input_data['waiting_time'] = 60
        twdcp_object.input_data['maximum_time'] = 60
        routing.AddDimension(
            transit_callback_index,
            twdcp_object.input_data['waiting_time'],  # allow waiting time
            twdcp_object.input_data['maximum_time'],  # maximum time per vehicle
            False,  # noqa: WPS425 Don't force start cumul to zero.
            dimension_name,
        )
        time_dimension = routing.GetDimensionOrDie(dimension_name)
        # Add time window constraints for each location except depot.
        for location_idx, time_window in enumerate(twdcp_object.input_data['time_windows']):
            if location_idx == 0:
                continue
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        # Add time window constraints for each vehicle start node.
        for vehicle_id in range(twdcp_object.input_data['num_vehicles']):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                twdcp_object.input_data['time_windows'][0][0],
                twdcp_object.input_data['time_windows'][0][1],
            )

        # Add resource constraints at the depot.
        solver = routing.solver()
        intervals = []
        for vehicle in range(twdcp_object.input_data['num_vehicles']):
            # Add time windows at start of routes
            intervals.append(
                solver.FixedDurationIntervalVar(
                    time_dimension.CumulVar(routing.Start(vehicle)),
                    twdcp_object.input_data['vehicle_load_time'],
                    'depot_interval',
                ),
            )
            # Add time windows at end of routes and
            # Instantiate route start and end times to produce feasible times.
            intervals.append(
                solver.FixedDurationIntervalVar(
                    time_dimension.CumulVar(routing.End(vehicle)),
                    twdcp_object.input_data['vehicle_unload_time'],
                    'depot_interval',
                ),
            )
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(vehicle)),
            )
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.End(vehicle)),
            )

        depot_usage = [1 for _ in range(len(intervals))]
        solver.Add(
            solver.Cumulative(intervals, depot_usage, twdcp_object.input_data['depot_capacity'], 'depot'),
        )

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            twdcp_object.print_solution(manager, routing, solution)
        else:
            print('No solution found !')
