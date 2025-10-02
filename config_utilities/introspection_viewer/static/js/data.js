// Data parsing utilities.
// This uses the Introspection Tree Data Structure.
// ---------- Original Fields: Fields from the original data that may not be modified (present if set) ----------
// - history: list of events
// - key: Name of the key in the parent map (if any)
// - map: child nodes, stored as array to preserve order (child nodes will have a 'key' field)
// - list: list of child nodes 
// -------------- Processed Fields:  Fields set by the processing that are used to render. These need to be overwritten or cleared to get the desired rendering --------------
// - value (str): the value to display if the node is a leaf
// - color: color of the text
// - tooltip: tooltip text

// ========== Queries on the original data structure ==========

// Global settings:
const settings = {
    indent_width: 4, // Number of spaces per indent level
    spaces_before_value: 2, // Number of spaces before the value in a leaf node
}

// Check whether a value was (set or updated)
function valueSet(event) {
    if (event.type !== "s" && event.type !== "u") {
        return false;
    }
    return event.val !== undefined;
}

// Check whether a value was modified (set, unset, updated, removed)
function valueModified(event) {
    if (event.type == "r") {
        return true;
    }
    return valueSet(event);
}

function isGetEvent(event) {
    return event.type === "g" || event.type === "d" || event.type === "e" || event.type === "a";
}

function isLeaf(node) {
    return node.value && node.value !== "";
}

function isListOfLeaves(node) {
    return node.list ? node.list.every(isLeaf) : false;
}

function sortDataAlphabetically(node) {
    if (node.map) {
        node.map = Object.values(node.map)
            .sort((a, b) => a.key.localeCompare(b.key));
        for (const child of Object.values(node.map)) {
            sortDataAlphabetically(child);
        }
    }
    if (node.list) {
        for (const child of node.list) {
            sortDataAlphabetically(child);
        }
    }
}


// Get the value at the end of the setting process. If a value is never set but read, return the last read value instead.
function getLatestValue(history) {
    let result = "";
    for (const event of history) {
        if (valueModified(event)) {
            result = event.val;
            continue;
        }
        if (isGetEvent(event) && result === "" && event.val !== undefined) {
            result = event.val;
        }
    }
    return result;
}

// ========== Processing Functions ==========

// Default: reset the display fields and render the latest value.
function defaultParsingFn(node) {
    if (node.history) {
        node.value = getLatestValue(node.history);
    } else if (node.value) {
        delete node.value;
    }
    delete node.color;
    delete node.tooltip;
}

// Processing functions to write the data to be displayed from the original data.
function createDisplayData(node, parsingFn = defaultParsingFn) {
    // Recursively process the data tree to add display fields.
    parsingFn(node);
    if (node.map) {
        for (const child of Object.values(node.map)) {
            createDisplayData(child, parsingFn);
        }
    }
    if (node.list) {
        for (const child of node.list) {
            createDisplayData(child, parsingFn);
        }
    }
}

// ========== Rendering Functions ==========

// Turn the processed data into HTML for display.
function renderDisplayData(node) {
    result = renderDisplayDataRec(node, 0);
    if (result.startsWith("<br>")) {
        result = result.slice(4);
    }
    return `<span class="yaml-value">${result}</span>`;
}

// Helper function for renderDisplayData.
function renderDisplayDataRec(node, indent = 0) {
    let result = "";
    if (!!node.value && !!node.map && !!node.list) {
        return result;
    }

    // Render list children if they exist.
    const indent_str = " ".repeat(indent * settings.indent_width);
    if (node.list) {
        if (isListOfLeaves(node)) {
            // Render a list of leaves in flow style: [item1, item2, item3]
            let items = node.list.map(item => formatValue(item.value, item));
            result += " ".repeat(settings.spaces_before_value) + `[${items.join(", ")}]`;
        } else {
            for (i = 0; i < node.list.length; i++) {
                // Render list of non-leaves in block style:
                let child_html = renderDisplayDataRec(node.list[i], indent + 1);
                if (child_html.startsWith("<br>")) {
                    child_html = child_html.slice(settings.indent_width * (indent + 1) + 4);
                }
                if (child_html) {
                    result += "<br>" + indent_str + "-   " + child_html;
                }
            }
        }
    }

    // Render map children if they exist.
    if (node.map) {
        for (const child of Object.values(node.map)) {
            if (isLeaf(child)) {
                result += "<br>" + indent_str + formatValue(child.key + ":" + " ".repeat(settings.spaces_before_value) + child.value, child);
            } else {
                const child_html = renderDisplayDataRec(child, indent + 1);
                if (child_html) {
                    result += "<br>" + indent_str + formatValue(child.key + ":", child) + child_html;
                }
            }
        }
    }
    return result;
}

// Render a single value of a node including the rendering description.
function formatValue(value, options) {
    let style = "";
    if (options.color) {
        style += `color: ${options.color};`;
    }
    let html = `<span class="yaml-value`;
    if (options.tooltip) {
        html += ` tooltip`;
    }
    html += `"`;
    if (style !== "") {
        html += ` style="${style}"`;
    }
    if (options.identifiers) {
        html += ` identifiers="${options.identifiers.join(',')}"`;
    }
    html += `>${value}`;
    if (options.tooltip) {
        html += `<span class="tooltiptext">${options.tooltip}</span>`;
    }

    // If no special styling, return plain text node.
    return html + `</span>`;
}