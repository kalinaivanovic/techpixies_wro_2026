"""
Swappable strategy implementations (Strategy pattern).

Each strategy type has an ABC and one or more implementations.
Pass the desired implementation to SensorFusion or StateMachine.
"""

from .clustering import (
    ClusteringStrategy,
    DetectedObject,
    OpenCVClustering,
    RawScanClustering,
)
from .wall_follow import (
    WallFollowStrategy,
    ProportionalWallFollow,
)
from .corner import (
    CornerStrategy,
    LidarCornerDetection,
)
from .avoidance import (
    AvoidanceStrategy,
    ProportionalAvoidance,
)
from .parking import (
    ParkingStrategy,
)
from .wall_detection import (
    WallDetectionStrategy,
    AverageWallDetection,
    ClusteringWallDetection,
)
