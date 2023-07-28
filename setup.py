#!/usr/bin/env python

"""The setup script."""

from pathlib import Path

from setuptools import find_packages, setup

with open('README.rst') as readme_file:
    readme = readme_file.read()

with open('HISTORY.rst') as history_file:
    history = history_file.read()


def get_requiremenst(path: Path) -> list[str]:
    with path.open() as requirements_txt:
        lines = list(requirements_txt)
        libraries = [line.strip() for line in lines]
    return libraries


# requirements = get_requiremenst(Path(__file__).parent.resolve() / 'requirements' / 'requirements.txt')
requirements = []
test_requirements = []

setup(
    author='Marco Di Rocco',
    author_email='marco.dirocco@uniroma1.it',
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: Italian',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
    description='The project contains some solution methods for optimization problems.',
    entry_points={
        'console_scripts': [
            'ort_optimization=ort_optimization.cli:main',
        ],
    },
    install_requires=requirements,
    license='MIT license',
    long_description=f'{readme}\n\n{history}',
    include_package_data=True,
    keywords='ort_optimization',
    name='ort_optimization',
    packages=find_packages(include=['ort_optimization', 'ort_optimization.*']),
    test_suite='tests',
    tests_require=test_requirements,
    url='https://github.com/MDiRocco/ort_optimization',
    version='0.1.2',
    zip_safe=False,
)
