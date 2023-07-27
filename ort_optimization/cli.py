"""Console script for ort_optimization."""
import click

from ort_optimization.tsp import TSP


@click.group()
def cli():
    """Entrypoint for cli."""
    pass  # noqa: WPS420


@cli.command()
# @click.option('--count', default=1, help='number of greetings')
@click.argument('file_path')
def tsp(file_path):
    """Options, Arguments and call main.

    Args:
        file_path: Path to the data input.

    Returns:
        A dictionary with the bands data.
    """
    return TSP.solve(file_path)
