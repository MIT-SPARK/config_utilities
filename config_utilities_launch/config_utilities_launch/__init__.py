"""Main entry point for the config_utilities_launch package."""

from launch.action import Action
from launch.frontend import expose_action, Entity, Parser
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node

from typing import Optional, List


def _substitute(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


@expose_action("configured_node")
class ConfiguredNode(Node):
    """Configurable node via ytt."""

    def __init__(self, *args, config_files=None, config_options=None, **kwargs):
        """Construct a node."""
        super().__init__(*args, **kwargs)
        self._config_files = config_files if config_files is not None else []
        self._config_options = config_options if config_options is not None else []

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse a node from a launch file."""
        _, kwargs = super().parse(entity, parser)
        files = entity.get_attr("config_files", optional=True, data_type=List[str])
        if files is not None:
            kwargs["config_files"] = [parser.parse_substitution(x) for x in files]

        opts = entity.get_attr("config_options", optional=True, data_type=List[str])
        if opts is not None:
            kwargs["config_options"] = [parser.parse_substitution(x) for x in opts]

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Call ytt and then call the underlying execute."""
        self._config_files = [_substitute(context, x) for x in self._config_files]
        self._config_options = [_substitute(context, x) for x in self._config_options]
        args = ["ytt"] + []
        return super().execute(context)
