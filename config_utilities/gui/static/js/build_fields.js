function buildField(data) {

}

function buildFields(data) {
    var tab = document.getElementById("config-table");

    for (var i = 0; i < data.length; i++) {
        var d = data[i];
        html = "<tr class='config-row'>";

        // Label.
        html += "<td class='config-label' padding-left='" + (d.indent * 3) + "px'>" + d.name + "</td>";

        // Field.
        html += "<td class='config-field'>";
        html += "<input type='text' action='/' method='post' id='field-" + d.id + "' value='" + d.value + "'></td>";
        // Default.
        html += "<td class='config-default'>" + d.default + "</td>";

        html += "<td>" + d.id + "</td>";
        html += "<td>" + d.indent + "</td>";

        // Finish.
        html += "</tr>";
        tab.innerHTML += html;
    }
}