"""Example tests for the Thesis project."""

import sys
from pathlib import Path

# Add src to path for testing
src_path = Path(__file__).parent.parent / "src"
sys.path.insert(0, str(src_path))

from src.example import hello_world, add_numbers


def test_hello_world():
    """Test the hello_world function."""
    assert hello_world() == "Hello, World!"
    assert hello_world("Alice") == "Hello, Alice!"


def test_add_numbers():
    """Test the add_numbers function."""
    assert add_numbers(2, 3) == 5
    assert add_numbers(0, 0) == 0
    assert add_numbers(-1, 1) == 0

