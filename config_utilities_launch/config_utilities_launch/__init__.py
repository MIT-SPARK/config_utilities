"""Main entry point for the config_utilities_launch package."""

from launch.action import Action
from launch.frontend import expose_action, Entity, Parser
from launch.launch_context import LaunchContext
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.actions import Node

from typing import Optional, List
from dataclasses import dataclass


@dataclass
class ConfigFile:
    """Quick class representing a file."""

    filepath: str  # TODO(nathan) this isn't just a str
    ns: Optional[str] = None  # TODO(nathan) this isn't just a str
    allow_substs: bool = False


@dataclass
class ConfigField:
    """Quick class representing a single config value at a name."""

    name: str
    value: str  # TODO(nathan) this isn't just a str
    allow_substs: bool = False


def _substitute(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _parse_config_section(section, parser):
    name_attr = section.get_attr("name", optional=True)
    from_attr = section.get_attr("from", optional=True)
    allow_attr = section.get_attr("allow_substs", optional=True)
    ns_attr = section.get_attr("ns", optional=True) if from_attr is not None else None
    value_attr = (
        section.get_attr("value", optional=True) if name_attr is not None else None
    )

    if from_attr is not None and name_attr is not None:
        raise RuntimeError(
            "Invalid config field: must either be a named param or a file"
        )

    if from_attr is None and name_attr is None:
        raise RuntimeError("Invalid config field: must have a 'name' or 'from' tag")

    section.assert_entity_completely_parsed()
    # TODO(nathan) validate this parses bools correctly
    allow_substs = False if allow_attr is not None else bool(allow_attr)

    if from_attr is not None:
        from_attr = parser.parse_substitution(from_attr)
        if ns_attr is not None:
            ns_attr = parser.parse_substitution(ns_attr)

        return ConfigFile(filepath=from_attr, ns=ns_attr, allow_substs=allow_substs)
    else:
        value_attr = parser.parse_substitution(value_attr)
        return ConfigField(name=name_attr, value=value_attr, allow_substs=allow_substs)


def _parse_config_list(config, parser):
    return [_parse_config_section(x, parser) for x in config]


@expose_action("configured_node")
class ConfiguredNode(Node):
    """Extension of Node that allows for collating arbitrary YAML files."""

    def __init__(self, *args, config=None, **kwargs):
        """Construct a node."""
        super().__init__(*args, **kwargs)
        self._config = config

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse a node from a launch file."""
        _, kwargs = super().parse(entity, parser)
        config = entity.get_attr("config", optional=True, data_type=List[Entity])
        if config is not None:
            kwargs["config"] = _parse_config_list(config, parser)

        return cls, kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Collate parsed config entries and pass to node."""
        print(self._config)
        self._config = [_substitute(context, x) for x in self._config]
        print(self._config)
        return super().execute(context)
