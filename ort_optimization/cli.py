"""Console script for ort_optimization."""
import click

from ort_optimization.ort_optimization import bis, ort


@click.command()
@click.option('--count', default=1, help='number of greetings')
@click.argument('names')
def main(count, names):
    """Options, Arguments and call main.

    Args:
        count: Name of the downloaded files.
        names: List of bands to manage.

    Returns:
        A dictionary with the bands data.
    """
    for _ in range(count):
        click.echo(f'Hello {names}!')
        bis()
    return ort()
