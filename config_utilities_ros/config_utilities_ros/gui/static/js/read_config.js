function post(url, data = {}) {
    // Post data to the server and reload the page.
    $.ajax({
        type: "POST",
        url: url,
        dataType: "json",
        data: data,
    });
    location.reload(true);
}

function readConfig() {
    const tab = document.getElementById("config-table");
    const rows = tab.getElementsByClassName("config-row");
    var data = {};
    for (var i = 0; i < rows.length; i++) {
        const row = rows[i];
        const values = row.getElementsByClassName("config-field-td");
        if (values.length == 0) {
            data[i] = "continue";
            continue;
        }
        const field = values[0].children[0];
        var id = field.id.split("-")[1];
        data[id] = field.value;
    }
    post("/submit", data);
}
