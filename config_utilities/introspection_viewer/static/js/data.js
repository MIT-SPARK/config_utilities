// Data parsing utilities.

function valueModified(event) {
    if (event.type !== "s" && event.type !== "u") {
        return false;
    }
    return event.val !== undefined;
}

function isGetEvent(event) {
    return event.type === "g" || event.type === "d" || event.type === "e";
}


// Create a nodes tree of the same structure as the data, with optional fields specifying the rendering properties:
// - value (str): if the node is a leaf, this is the string to display
// - map (dict): if the node is a map, this is a dict of child nodes
// - list (list): if the node is a list, this is a list of child nodes
// -------------- optional fields for rendering --------------
// - color: color of the text
// - tooltip: tooltip text

function getLatestValue(history) {
    result = "";
    for (const event of history) {
        if (valueModified(event)) {
            result = event.val;
            continue;
        }
        if (event.type == "r") {
            result = "";
            continue;
        }
        if (isGetEvent(event) && result == "" && event.val !== undefined) {
            result = event.val;
        }
    }
    return result;
}

function displayData(node, getValueFn = getLatestValue) {
    result = displayDataRec(node, 0, getValueFn);
    if (result.startsWith("<br>")) {
        result = result.slice(4);
    }
    return result;
}

function displayDataRec(node, indent = 0, getValueFn) {
    result = "";
    if (!node.history && !node.map && !node.list) {
        return result;
    }
    // Render the value if it exists.
    if (node.history) {
        const value = getValueFn(node.history);
        result += "&nbsp; " + formatValue(value, node);
    }

    // Render list children if they exist.
    indent_str = "&nbsp;".repeat(indent * 2);
    if (node.list) {
        for (i = 0; i < node.list.length; i++) {
            // TODO: check for leaf-only lists and render in flow style.
            result += "<br>" + indent_str + `[${i}]:` + displayDataRec(node.list[i], indent + 1, getValueFn);
        }
    }

    // Render map children if they exist.
    if (node.map) {
        for (const [key, child] of Object.entries(node.map)) {
            result += "<br>" + indent_str + `${key}:` + displayDataRec(child, indent + 1, getValueFn);
        }
    }
    return result;
}



/// Backup

function isLeaf(data) {
    return data.value !== undefined;
}

function isListOfLeaves(data) {
    if (!data.list) return false;
    return data.list.every(isLeaf);
}

function formatValue(value, options = {}) {
    // Render a single line of a value or key.
    let html = `<span class="yaml-value"`;
    let style = "";
    if (options.color) {
        style += `color: ${options.color};`;
    }
    if (options.tooltip) {
        style += `border-bottom: 1px dotted ${options.color || 'black'}; cursor: help;`;
        return html + ` style="${style}" title="${options.tooltip}">${value}</span>`;
    }
    if (style !== "") {
        html += ` style="${style}"`;
    }
    // If no special styling, return plain text node.
    return html + `>${value}</span>`;
}

function formatLine(key, valueHtml, indent) {
    // Render a single line of a key-value pair with indentation.
    const indentPx = indent * 20;
    const lineDiv = document.createElement("div");
    lineDiv.style.paddingLeft = `${indentPx}px`;
    if (key !== null) {
        const keySpan = document.createElement("span");
        keySpan.className = "yaml-value";
        keySpan.textContent = `${key}: `;
        lineDiv.appendChild(keySpan);
    }
    if (typeof valueHtml === "string") {
        lineDiv.innerHTML += valueHtml;
    } else {
        lineDiv.appendChild(valueHtml);
    }
    return lineDiv;
}

function formatListFlow(data) {
    // Render a list of leaves in flow style: [item1, item2, item3]
    let items = data.list.map(item => formatValue(item.value, item));
    return formatValue(`[${items.join(", ")}]`, data);
}


function renderHighlightableText(text, identifier, highlightClass = "highlight") {
    // Create a span with data-identifier attribute
    const span = document.createElement("span");
    span.textContent = text;
    span.setAttribute("data-identifier", identifier);

    // Mouse enter: highlight all elements with same identifier
    span.addEventListener("mouseenter", () => {
        document.querySelectorAll(`[data-identifier="${identifier}"]`).forEach(el => {
            el.classList.add(highlightClass);
        });
    });

    // Mouse leave: remove highlight
    span.addEventListener("mouseleave", () => {
        document.querySelectorAll(`[data-identifier="${identifier}"]`).forEach(el => {
            el.classList.remove(highlightClass);
        });
    });

    return span;
}

// Example CSS to add to your stylesheet:
// .highlight { background-color: yellow; }
