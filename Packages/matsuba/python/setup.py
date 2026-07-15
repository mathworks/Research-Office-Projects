from setuptools import setup, find_packages

setup(
    name="matlab_mitsuba",
    version="0.1.0",
    packages=find_packages(),
    install_requires=["mitsuba", "numpy"],
    description="Python adapter for MATSUBA — MATLAB wrapper for Mitsuba 3",
)
