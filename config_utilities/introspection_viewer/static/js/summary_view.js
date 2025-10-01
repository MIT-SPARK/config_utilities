function updateDisplay() {
    if (window.introSettings.current_view === "summary") {
        setupSummaryView();
    }
    // Future views can be added here.
}

function summaryBtnClicked(button) {
    if (window.introSettings.current_view !== "summary") {
        window.introSettings.current_view = "summary";
        updateDisplay();
        button.className = "config-button button-selected";
    }
}

function optionButtonClicked(button, option) {
    window.introSettings[option] = !window.introSettings[option];
    button.classList.toggle("button-selected");
    updateDisplay();
}

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
    var data = window.introData;
    createDisplayData(data, n => parseSummaryDefault(n, legendEntries));
    summaryPanelContent.innerHTML = renderDisplayData(data);

    // Create and append the legend.
    const legend = createLegend(legendEntries);
    displayPaneDiv.appendChild(legend);

    if (summaryPanelContent.scrollHeight > summaryPanelContent.clientHeight) {
        legend.style.right = '3em';
    }
}

function parseSummaryDefault(node, legendEntries) {
    // Set values, colors, etc. for summary view.
    node.value = "";
    node.color = "";
    if (!node.history) {
        delete node.tooltip;
        delete node.identifiers;
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
        if (!!window.introSettings['show_unset'] && isGetEvent(event) && node.value === "" && event.val !== undefined) {
            node.value = event.val;
        }
    }
    if (was_set && node.color === "") {
        node.color = legendEntries[2].color;
    }
    // Render the history and keep all sources as identifiers.
    node.tooltip = displayHistory(node.history);
    node.identifiers = [];
    if (window.introSettings['show_sources']) {
        for (const event of node.history) {
            if (!node.identifiers.includes(event.by)) {
                node.identifiers.push(event.by);
            }
        }
    }
}


