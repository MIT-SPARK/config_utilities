function post(url, data = {}) {
    // Post data to the server and reload the page.
    $.ajax({
        type: "POST",
        url: url,
        dataType: "json",
        data: data,
    });
}

function readConfig() {
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
    post("/submit", data);
}

function onVirtualConfigChange(element) {
    // Update the config name and type based on the selected virtual config.
    // const id = element.id.substring(element.id.indexOf("-") + 1);
    // const name = element.options[element.selectedIndex].text;
    // field.value = name;
    // field.width = element.value.length;

    data = {};
    data["test"] = "virtual config selected"
    // data[id] = element.value;
    post("/submit", data);
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