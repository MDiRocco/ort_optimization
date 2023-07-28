"""Console script for ort_optimization."""
import sys

import click

from ort_optimization.tsp import TSP


@click.group()
@click.version_option()
def main():
    """Entrypoint for cli."""
    pass  # noqa: WPS420


@main.command()
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
