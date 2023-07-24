#!/usr/bin/env python

"""The setup script."""

from setuptools import setup, find_packages

with open('README.rst') as readme_file:
    readme = readme_file.read()

with open('HISTORY.rst') as history_file:
    history = history_file.read()

requirements = [ ]

test_requirements = [ ]

setup(
    author="Marco Di Rocco",
    author_email='marco.dirocco@uniroma1.it',
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],
    description="The project contains some solution methods for optimization problems.",
    entry_points={
        'console_scripts': [
            'ort_optimization=ort_optimization.cli:main',
        ],
    },
    install_requires=requirements,
    license="MIT license",
    long_description=readme + '\n\n' + history,
    include_package_data=True,
    keywords='ort_optimization',
    name='ort_optimization',
    packages=find_packages(include=['ort_optimization', 'ort_optimization.*']),
    test_suite='tests',
    tests_require=test_requirements,
    url='https://github.com/MDiRocco/ort_optimization',
    version='0.1.0',
    zip_safe=False,
)
