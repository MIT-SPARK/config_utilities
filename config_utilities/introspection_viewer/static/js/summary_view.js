
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

    const panelContent = document.createElement("div");
    panelContent.id = "summary-panel-content";
    panelContent.style.maxHeight = `calc(100vh - ${displayPane.getBoundingClientRect().top}px - 10em)`;
    panelContent.style.overflowY = "auto";
    panelContent.style.width = "100%";
    panelContent.style.minWidth = "25%";
    panelContent.style.overflowY = "auto";
    panelContent.style.overflowX = "auto";
    panelContent.style.flexGrow = "1";
    panelContent.style.flexShrink = "1";
    displayPaneDiv.style.position = "relative";

    // panelContent.style.minWidth = "300px";
    panelContent.style.boxSizing = "border-box";

    displayPaneDiv.appendChild(panelContent);
    displayPane.appendChild(displayPaneDiv);

    // Render the data.
    renderSummaryDefault(displayPaneDiv, panelContent);
}

const eventColors = {
    "s": '#40a544ff',          // Set
    "u": '#1ad3caff',          // Update
    "n": '#379becff',            // SetNonModified
    "f": '#f44336',            // SetFailed
    "g": '#40a544ff',          // Get
    "d": '#379becff',          // GetDefault
    "a": '#666666',            // GetAbsent
    "e": '#f44336',          // GetError
    "r": '#ff9800',            // Remove
};

const eventNames = {
    "s": 'Set',
    "u": 'Updated',
    "n": 'Set (not modified)',
    "f": 'Set Failed',
    "g": 'Read',
    "d": 'Read (default)',
    "a": 'Read (absent)',
    "e": 'Read Failed',
    "r": 'Removed',
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
    var sources = window.introspectionData.sources;
    createDisplayData(data, n => parseSummaryDefault(n, legendEntries, sources));
    summaryPanelContent.innerHTML = renderDisplayData(data);

    // Create and append the legend.
    const legend = createLegend(legendEntries);
    displayPaneDiv.appendChild(legend);
    if (summaryPanelContent.scrollHeight > summaryPanelContent.clientHeight) {
        legend.style.right = '3em';
    }
}

function historyToTooltip(history, sources) {
    if (!history || !Array.isArray(history)) return "";
    // Create an invisible table with each event as a row
    let html = `<div style="margin: 0.5em;"><strong>History</strong><table style="margin: 0 auto;"><tr>
        <th style="padding-right: 2em;">Event</th>
        <th style="padding-right: 2em;">By</th>
        <th style="padding-right: 2em;">Value</th>
        </tr>`;
    for (const event of history) {
        html += `<tr>
            <td style="padding-right: 2em;"><span style="color: ${eventColors[event.type]}; font-weight: bold;">${eventNames[event.type]}</span></td>
            <td style="padding-right: 2em;">${getSourceName(sources, event)}</td>
            <td style="padding-right: 2em;">${event.val !== undefined ? event.val : ""}</td>
        </tr>`;
    }
    html += `</table></div>`;
    return html;
}

function parseSummaryDefault(node, legendEntries, sources, includeUnset = true) {
    // Set values, colors, etc. for summary view.
    node.value = "";
    node.color = "";
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
    node.tooltip = historyToTooltip(node.history, sources);
}


