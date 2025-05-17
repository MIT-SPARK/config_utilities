function buildField(field) {
    // action='/' method='post'
    // html += " onchange=\"post('/set', {'" + field.id + "': this.value})\"";
    info = field.input_info;

    // Element type.
    if (info.type == "options") {
        html = "<select";
    } else {
        html = "<input";
    }

    // Common attributes.
    html += " class='config-field' id='field-" + field.id + "' value='" + field.value + "'";

    // Switch types.
    if (info.type == "bool") {
        if (field.value) {
            html += " checked";
        }
        html += " type='checkbox' style='width: auto;'";
    } else if (info.type == "options") {
        html += ">";
        for (var i = 0; i < info.options.length; i++) {
            var d = info.options[i];
            html += "<option value='" + d + "'";
            if (d == field.value) {
                html += " selected";
            }
            html += ">" + d + "</option>";
        }
        return html + "</select>";
    } else if (info.type == "string") {
        html += " type='text'";
    } else if (info.type.includes("int")) {
        html += " type='number' step='1'";
        new_min = NaN;
        new_max = NaN;
        if (info.type == "int8") {
            new_min = -128;
            new_max = 127;
        }
        if (info.type == "int16") {
            new_min = -32768;
            new_max = 32767;
        }
        if (info.type == "int32") {
            new_min = -2147483648;
            new_max = 2147483647;
        }
        if (info.type == "int64") {
            new_min = -9223372036854775808n;
            new_max = 9223372036854775807n;
        }
        if (info.type == "uint8") {
            new_min = 0;
            new_max = 255;
        }
        if (info.type == "uint16") {
            new_min = 0;
            new_max = 65535;
        }
        if (info.type == "uint32") {
            new_min = 0;
            new_max = 4294967295;
        }
        if (info.type == "uint64") {
            new_min = 0;
            new_max = 18446744073709551615n;
        }
        if ("min" in info) {
            new_min = Math.max(new_min, info.min);
        }
        if ("max" in info) {
            new_max = Math.min(new_max, info.max);
        }
        if (!isNaN(new_min)) {
            html += " min='" + new_min + "'";
        }
        if (!isNaN(new_max)) {
            html += " max='" + new_max + "'";
        }
    } else if (info.type.includes("float")) {
        // Ignore floating point limits for now.
        html += " type='number' step='any'";
        if ("min" in info) {
            html += " min='" + info.min + "'";
        }
        if ("max" in info) {
            html += " max='" + info.max + "'";
        }
    } else {
        // Yaml type. Unknown defaults to yaml.
        html += " type='text'";
    }
    return html + ">";
}

function buildFields(data) {
    var tab = document.getElementById("config-table");

    for (var i = 0; i < data.length; i++) {
        var d = data[i];
        html = "<tr class='config-row'>";

        // Label.
        html += "<td class='config-label'"
        if (d.type == "config") {
            html += " colspan='3'";
        }
        html += ">";

        // Indent.
        for (var j = 0; j < d.indent; j++) {
            html += "&nbsp;&nbsp;&nbsp;&nbsp;";
        }
        html += d.name;

        // Unit.
        if ('unit' in d) {
            html += " [" + d.unit + "]";
        }

        // Config headers.
        if (d.type == "config") {
            html += " [" + d.config_name + "]:</td>";
            tab.innerHTML += html;
            continue;
        }

        html += ":</td>";

        // Field.
        html += "<td class='config-field-td'>" + buildField(d) + "</td>";

        // Default.
        html += "<td class='config-default'>" + d.default + "</td>";

        // Finish.
        html += "</tr>";
        tab.innerHTML += html;
    }
}