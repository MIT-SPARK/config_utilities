
function setupSummaryView() {

    // Format the display pane
    const displayPane = document.getElementById("displayPane");
    displayPane.innerHTML = ""; // Clear previous content

    const heading = document.createElement("h2");
    heading.textContent = "Summary View";
    displayPane.appendChild(heading);

    const displayPaneDiv = document.createElement("div");
    displayPaneDiv.className = "display-pane";
    displayPaneDiv.style.display = "flex";
    displayPaneDiv.style.gap = "2em";

    const summaryPanelContent = document.createElement("div");
    summaryPanelContent.id = "summary-panel-content";
    summaryPanelContent.style.flex = "1 1 0";

    displayPaneDiv.appendChild(summaryPanelContent);
    displayPane.appendChild(displayPaneDiv);

    // Render the data.
    renderSummaryDefault(displayPaneDiv, summaryPanelContent);
}

const eventColors = {
    "s": '#40a544ff',          // Set
    "u": '#379becff',          // Update
    "n": '#ff9800',            // SetNonModified
    "f": '#f44336',            // SetFailed
    "g": '#34c0caff',          //  Get
    "d": '#fae20fff',          // GetDefault
    "a": '#666666',            // GetAbsent
    "e": '#c5108fff',          // GetError
    "r": '#000000',            // Remove
};

function renderSummaryDefault(displayPaneDiv, summaryPanelContent) {
    // Definitions.
    const legendEntries = [
        { color: '#40a544ff', label: 'Parsed' },
        { color: '#379becff', label: 'Parsed (is Default)' },
        { color: '#ff9800', label: 'Set but not read' },
        { color: '#f44336', label: 'Parsing failed' },
        { color: '#666666', label: 'Read but not set' },
        { color: '#9b40c5ff', label: 'Multiple conflicting labels' },
    ];

    // Parse the data.
    var data = window.introspectionData.data;
    createDisplayData(data, n => parseSummaryDefault(n, legendEntries, false));
    summaryPanelContent.innerHTML = renderDisplayData(data);

    // Create and append the legend.
    displayPaneDiv.appendChild(createLegend(legendEntries));
}

function parseSummaryDefault(node, legendEntries, includeUnset = true) {
    // Set values, colors, etc. for summary view.
    node.value = "";
    node.color = "";
    node.tooltip = "asdjjas";
    if (!node.history) {
        delete node.value;
        delete node.color;
        delete node.tooltip;
        return;
    }
    let was_set = false;
    const set_color = c => {
        if (node.color === "") {
            node.color = c;
        } else if (node.color !== c) {
            node.color = legendEntries[5].color; // Conflict color
        }
    };
    for (const event of node.history) {
        // Get the color for the latest event.
        if (event.type == "g") {
            set_color(legendEntries[0].color);
        } else if (event.type == "d") {
            set_color(legendEntries[1].color);
        } else if (event.type == "e") {
            set_color(legendEntries[3].color);
        } else if (event.type == "a") {
            set_color(legendEntries[4].color);
        }
        // Get the display value.
        if (valueModified(event)) {
            node.value = event.val;
            was_set = true;
            continue;
        }
        if (includeUnset && isGetEvent(event) && node.value === "" && event.val !== undefined) {
            node.value = event.val;
        }
    }
    if (was_set && node.color === "") {
        node.color = legendEntries[2].color;
    }
}


