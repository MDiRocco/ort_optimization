"""Console script for ort_optimization."""
import click

from ort_optimization.cvrp import CVRP
from ort_optimization.pdp import PDP
from ort_optimization.tsp import TSP
from ort_optimization.twcp import TWCP
from ort_optimization.twdcp import TWDCP
from ort_optimization.vrp import VRP


@click.group()
@click.version_option()
def main():
    """Entrypoint for cli."""
    pass  # noqa: WPS420


@main.command()
@click.argument('file_path')
def cvrp(file_path):
    """Solve the Vehicles Routing Problem (VRP).

    Args:
        file_path: Path to the data input.

    Returns:
        Routes for the vehicles.
    """
    return CVRP.solve(file_path)


@main.command()
@click.argument('file_path')
def pdp(file_path):
    """Solve the Vehicles Routing Problem (VRP).

    Args:
        file_path: Path to the data input.

    Returns:
        Routes for the vehicles.
    """
    return PDP.solve(file_path)


@main.command()
# @click.option('--count', default=1, help='number of greetings')
@click.argument('file_path')
def tsp(file_path):
    """Solve the Traveling Salesperson Problem (TSP).

    Args:
        file_path: Path to the data input.

    Returns:
        A Route for the vehicle.
    """
    return TSP.solve(file_path)


@main.command()
@click.argument('file_path')
def twcp(file_path):
    """Solve the Vehicles Routing Problem (VRP).

    Args:
        file_path: Path to the data input.

    Returns:
        Routes for the vehicles.
    """
    return TWCP.solve(file_path)


@main.command()
@click.argument('file_path')
def twdcp(file_path):
    """Solve the Vehicles Routing Problem (VRP).

    Args:
        file_path: Path to the data input.

    Returns:
        Routes for the vehicles.
    """
    return TWDCP.solve(file_path)


@main.command()
@click.argument('file_path')
def vrp(file_path):
    """Solve the Vehicles Routing Problem (VRP).

    Args:
        file_path: Path to the data input.

    Returns:
        Routes for the vehicles.
    """
    return VRP.solve(file_path)
