function setupSummaryView(displayPanel, panelContent) {
    // Render the data.
    renderSummaryDefault(displayPanel, panelContent);
}


function renderSummaryDefault(displayPanel, panelContent) {
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
    if (window.introSettings['sort'] === true) {
        data = JSON.parse(JSON.stringify(window.introData)); // Deep copy
        sortDataAlphabetically(data);
    }
    createDisplayData(data, n => parseSummaryDefault(n, legendEntries));
    panelContent.innerHTML = renderDisplayData(data);

    let upperBound = panelContent.getBoundingClientRect().top;
    let previousHeight = "0px";
    const panelHasScroll = panelContent.scrollHeight > panelContent.clientHeight;
    // Create and append the legend.
    if (window.introSettings['show_legend'] === true) {
        const legend = createLegend(legendEntries);
        legend.id = "legend";
        if (panelHasScroll) {
            legend.style.right = '3em';
        }
        displayPanel.appendChild(legend);
        upperBound = legend.getBoundingClientRect().bottom;
        previousHeight = `calc(${legend.getBoundingClientRect().height}px + 1em)`;
    }

    // Check if sources should be shown.
    if (window.introSettings['show_sources'] === true) {
        const sourcesLegend = createSourcesLegend();
        if (panelHasScroll) {
            sourcesLegend.style.right = '3em';
        }
        sourcesLegend.style.top = `calc(${previousHeight} + 1em)`;
        sourcesLegend.style.maxHeight = `calc(100vh - ${upperBound}px - 5em)`;
        sourcesLegend.style.overflowY = "auto";
        sourcesLegend.style.width = "100%";
        sourcesLegend.style.minWidth = "25%";
        sourcesLegend.style.overflowX = "auto";
        sourcesLegend.style.flexGrow = "1";
        sourcesLegend.style.flexShrink = "1";
        sourcesLegend.style.boxSizing = "border-box";
        displayPanel.appendChild(sourcesLegend);
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


