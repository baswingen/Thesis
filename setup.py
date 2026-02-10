"""Setup configuration for the Thesis project."""

from setuptools import setup, find_packages

try:
    with open("README.md", "r", encoding="utf-8") as fh:
        long_description = fh.read()
except FileNotFoundError:
    with open("README/README.md", "r", encoding="utf-8") as fh:
        long_description = fh.read()

with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="thesis",
    version="0.1.0",
    author="Bas Wingen",
    author_email="baswingen@gmail.com",
    description="Thesis project",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/baswingen/thesis",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
    python_requires=">=3.8,<3.13",  # Python 3.12 recommended for PyTorch/TensorFlow/Keras compatibility
    install_requires=requirements,
)

