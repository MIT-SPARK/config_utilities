

#include "config_utilities/printing.h"

#include "config_utilities/factory.h"

namespace config::internal {

void checkDefaultValues(MetaData& data, const MetaData& default_data) {
  for (internal::FieldInfo& info : data.field_infos) {
    if (info.subconfig_id >= 0) {
      // This means the field is a subconfig, check the entire subconfig.
      if (data.is_virtual_config) {
        // If the subconfig is variable and set, create the default object to compare values.
      }
      checkDefaultValues(data.sub_configs[info.subconfig_id], default_data.sub_configs[info.subconfig_id]);
      info.is_default = true;
      for (const internal::FieldInfo& sub_info : data.sub_configs[info.subconfig_id].field_infos) {
        if (!sub_info.is_default) {
          info.is_default = false;
          break;
        }
      }
    } else {
      // Check the field itself. NOTE(lschmid): Operator YAML::Node== checks for identity, not equality. Comparing the
      // formatted strings should be identical for default constructed configs. The clone is needed as for yaml nodes
      // are internally references and operator[] occasionally messes up things.
      YAML::Node data_node = YAML::Clone(data.data);
      YAML::Node default_node = YAML::Clone(default_data.data);
      if (!data_node[info.name] || !default_node[info.name]) {
        return;
      }
      if (internal::dataToString(data_node[info.name]) == internal::dataToString(default_node[info.name])) {
        info.is_default = true;
      }
    }
  }
}

}  // namespace config::internal
