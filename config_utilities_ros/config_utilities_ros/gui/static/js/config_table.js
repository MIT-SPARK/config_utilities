function post(url, data = {}) {
    // Post data to the server and reload the page.
    $.ajax({
        type: "POST",
        url: url,
        dataType: "json",
        data: data,
    });
}

function readConfigData() {
    const tab = document.getElementById("config-table");
    const rows = tab.getElementsByClassName("config-row");
    var data = {};
    for (var i = 0; i < rows.length; i++) {
        const row = rows[i];
        const values = row.getElementsByClassName("config-field-td");
        if (values.length == 1) {
            // Regular field.
            const field = values[0].children[0];
            const id = field.id.substring(field.id.indexOf("-") + 1)
            data[id] = field.value;
            continue;
        }
        const subconfigs = row.children[0].getElementsByClassName("subconfig-field");
        if (subconfigs.length >= 1) {
            // Get map keys for mapped configs and virtual config types.
            for (var j = 0; j < subconfigs.length; j++) {
                const field = subconfigs[j];
                const id = field.id.substring(field.id.indexOf("-") + 1)
                data[id] = field.value
            }
        }
    }
    return data;
}

function onAddDelete(id, action) {
    // First update the config data, then execute addition or deletion.
    var conf_data = readConfigData();
    conf_data["_id"] = id;
    conf_data["_action"] = action;

    post("/add_delete", conf_data);
}

function onVirtualConfigChange(id) {
    // element = document.getElementById(id);
    // const value = element.options[element.selectedIndex].text;
    // post("/msg", { "id": id, value: value });
}

// Function to make table columns resizable
function makeColumnsResizable(table) {
    const ths = table.querySelectorAll('th');
    const tableId = table.id || 'config-table';

    // Load saved widths
    const savedWidths = JSON.parse(localStorage.getItem(tableId + '-col-widths') || '[]');
    ths.forEach((th, i) => {
        if (savedWidths[i]) th.style.width = savedWidths[i];
    });

    ths.forEach((th, i) => {
        if (i === ths.length - 1) return; // skip last column
        const resizer = document.createElement('div');
        resizer.className = 'resizer';
        th.appendChild(resizer);

        let startX, startWidth;
        resizer.addEventListener('mousedown', function (e) {
            startX = e.pageX;
            startWidth = th.offsetWidth;
            document.documentElement.addEventListener('mousemove', mousemove);
            document.documentElement.addEventListener('mouseup', mouseup);
        });

        function mousemove(e) {
            const dx = e.pageX - startX;
            th.style.width = (startWidth + dx) + 'px';
        }
        function mouseup() {
            // Save all column widths
            const widths = Array.from(ths).map(th => th.style.width || th.offsetWidth + 'px');
            localStorage.setItem(tableId + '-col-widths', JSON.stringify(widths));
            document.documentElement.removeEventListener('mousemove', mousemove);
            document.documentElement.removeEventListener('mouseup', mouseup);
        }
    });
}

document.addEventListener('DOMContentLoaded', function () {
    const table = document.getElementById('config-table');
    if (table) makeColumnsResizable(table);
});