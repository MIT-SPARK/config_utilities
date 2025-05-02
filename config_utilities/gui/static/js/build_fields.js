function buildField(data) {

}

function buildFields(data) {
    var tab = document.getElementById("config-table");

    for (var i = 0; i < data.length; i++) {
        var d = data[i];
        html = "<tr class='config-row'>";

        // Label.
        html += "<td class='config-label'>"
        for (var j = 0; j < d.indent; j++) {
            html += "&nbsp;&nbsp;&nbsp;&nbsp;";
        }
        html += d.name;
        if ('unit' in d) {
            html += " [" + d.unit + "]";
        }
        html += ":</td>";

        // Field.
        html += "<td class='config-field'>";
        html += "<input type='text' action='/' method='post' id='field-" + d.id + "' value='" + d.value + "'></td>";
        // Default.
        html += "<td class='config-default'>" + d.default + "</td>";

        // Finish.
        html += "</tr>";
        tab.innerHTML += html;
    }
}