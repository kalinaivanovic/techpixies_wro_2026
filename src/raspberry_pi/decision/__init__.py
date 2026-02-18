"""
Decision Layer - What to do.

Contains:
- StateMachine: High-level state management
- Strategies: Algorithms for each state (wall follow, avoidance, etc.)
"""

from .state_machine import RobotState, StateMachine

__all__ = ["RobotState", "StateMachine"]
