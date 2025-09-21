function loadJsonFile(path) {
    return fetch(path)
        .then(response => {
            if (!response.ok) {
                throw new Error(`Failed to load file: ${response.statusText}`);
            }
            return response.json();
        });
}

// Create a nodes tree of the same structure as the data, with optional fields specifying the rendering properties:
// - value (str): if the node is a leaf, this is the string to display
// - map (dict): if the node is a map, this is a dict of child nodes
// - list (list): if the node is a list, this is a list of child nodes
// -------------- optional fields for rendering --------------
// - color: color of the text
// - tooltip: tooltip text


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
    if (style) {
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

function renderYamlLike(data, indent = 0) {
    if (isLeaf(data)) {
        return formatValue(data.value, data);
    }
    if (data.list) {
        if (isListOfLeaves(data)) {

        }


    }
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
