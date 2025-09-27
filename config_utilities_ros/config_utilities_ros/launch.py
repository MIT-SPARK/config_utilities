from typing import Sequence, Text
from launch.launch_context import LaunchContext
from launch.frontend import expose_substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution


@expose_substitution("config-utilities-context")
class ConfigUtilitiesContext(Substitution):
    """Custom substitution for prefixing exec with correct interperter."""

    def __init__(self):
        """Create a ConfigUtiltiesContext substitution."""
        super().__init__()

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse `ConfigUtilitiesContext` substitution."""
        if len(data):
            raise AttributeError("config-utilities-context takes no arguments")

        return cls, {}

    def describe(self) -> Text:
        """Return descriptor for this string."""
        return "ConfigUtilitiesContext()"

    def perform(self, context: LaunchContext) -> Text:
        """Render variables to args for config_utilities."""
        args = [f"{k}={v}" for k, v in context.launch_configurations.items()]
        args = [f"--config-utilities-var {x}" for x in args]
        return " ".join(args)
