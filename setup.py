from setuptools import setup, find_packages

setup(
    name="dynamic_body_sim",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "mujoco",
        "numpy",
    ],
    package_data={
        "actauted_pendulum": ["*.xml"],
    },
    description="Dynamic body simulation using MuJoCo",
    author="Aaron Xie",
    author_email="aaron@kscale.dev",
) 