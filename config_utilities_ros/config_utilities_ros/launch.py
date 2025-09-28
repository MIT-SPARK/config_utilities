import pathlib
import tempfile
from typing import Sequence, Text, Optional

from launch.launch_context import LaunchContext
from launch.frontend import expose_substitution
from launch.frontend.parse_substitution import parse_substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


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
        args = [f"--config-utilities-var '{x}'" for x in args]
        return " ".join(args)


@expose_substitution("resolve-file-substitutions")
class ResolveFileSubstitutions(Substitution):
    """Custom substitution for resolving all substitutions in a file."""

    def __init__(
        self,
        path: SomeSubstitutionsType,
        output: Optional[SomeSubstitutionsType] = None,
    ):
        """Create a substitution."""
        super().__init__()

        self.__path = normalize_to_list_of_substitutions(path)
        self.__output = output
        if self.__output:
            self.__output = normalize_to_list_of_substitutions(self.__output)

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        if len(data) < 1 or len(data) > 2:
            raise AttributeError(
                "resolve-file-substitutions takes 1-2 arguments")

        kwargs = {"path": data[0]}
        if len(data) > 1:
            kwargs["output"] = data[1]
        return cls, kwargs

    def describe(self) -> Text:
        """Return descriptor."""
        desc = "ResolveFileSubstitutions("
        desc += "path=" + ", ".join([sub.describe() for sub in self.__path])
        if self.__output:
            desc += ", "
            desc += "output=" + ", ".join(
                [sub.describe() for sub in self.__output])

        desc += ")"
        return desc

    def perform(self, context: LaunchContext) -> Text:
        """Open file and pass."""
        path = perform_substitutions(context, self.__path)
        path = pathlib.Path(path)
        if not path.exists():
            raise SubstitutionFailure(f"File not found: {path}")

        with path.open("r") as fin:
            contents = fin.read()

        contents = parse_substitution(contents)
        contents = perform_substitutions(context, contents)

        if self.__output:
            output = pathlib.Path(perform_substitutions(
                context, self.__output))
            try:
                with output.open("w") as fout:
                    fout.write(contents)
            except Exception as e:
                raise SubstitutionFailure(
                    f"Output file not valid: {output}: {e}")

            return output

        with tempfile.NamedTemporaryFile(mode="w", delete=False) as fout:
            fout.write(contents)
            return fout.name
