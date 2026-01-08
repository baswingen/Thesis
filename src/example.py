"""Example module for the Thesis project."""


def hello_world(name: str = "World") -> str:
    """
    A simple hello world function.
    
    Args:
        name: Name to greet (default: "World")
        
    Returns:
        A greeting string
    """
    return f"Hello, {name}!"


def add_numbers(a: float, b: float) -> float:
    """
    Add two numbers together.
    
    Args:
        a: First number
        b: Second number
        
    Returns:
        Sum of a and b
    """
    return a + b

