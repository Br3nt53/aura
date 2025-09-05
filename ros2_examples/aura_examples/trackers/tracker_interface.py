from abc import ABC, abstractmethod
from typing import Any, Dict, List


class BaseTracker(ABC):
    """Abstract interface for a multi-target tracker."""

    @abstractmethod
    def __init__(self, params: Dict[str, Any]):
        """Initializes the tracker with a dictionary of parameters."""
        pass

    @abstractmethod
    def update(
        self, detections: List[Dict[str, Any]], timestamp_sec: float
    ) -> List[Dict[str, Any]]:
        """Processes detections and returns the current state of active tracks."""
        pass
