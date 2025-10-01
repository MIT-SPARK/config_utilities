// ========== Constants ==========

const eventColors = {
    "s": '#40a544ff',          // Set
    "u": '#1ad3caff',          // Update
    "n": '#379becff',          // SetNonModified
    "f": '#f44336',            // SetFailed
    "g": '#40a544ff',          // Get
    "d": '#379becff',          // GetDefault
    "a": '#666666',            // GetAbsent
    "e": '#f44336',            // GetError
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

const byIcons = {
    "f": `<i class="fa-solid fa-file" style="font-size: 0.8em;"></i>`, // File
    "a": `<i class="fa-solid fa-terminal" style="font-size: 0.8em;"></i>`, // Argument
    "s": `<i class="fa-solid fa-rotate-left" style="font-size: 0.8em;"></i>`, // substitution
    "p": `<i class="fa-solid fa-code" style="font-size: 0.8em;"></i>`, // Programmatic
    "c": `<i class="fa-solid fa-cubes" style="font-size: 0.8em;"></i>`, // Config
}

// ========== General Rendering Tools ============

// Lookup the name of the source of an event.
function displaySource(event) {
    console.assert(event.by && event.by.length >= 2);
    const key = event.by.charAt(0);
    const index = parseInt(event.by.slice(1));
    const source_name = window.introSources[key][index];
    return byIcons[key] + " " + source_name;
}

function displayHistory(history) {
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
            <td style="padding-right: 2em;">${displaySource(event)}</td>
            <td style="padding-right: 2em;">${event.val !== undefined ? event.val : ""}</td>
        </tr>`;
    }
    html += `</table></div>`;
    return html;
}

// entries: list of {color, label}
function createLegend(entries) {
    const aside = document.createElement('aside');
    aside.className = 'legend-pane';

    const heading = document.createElement('h3');
    heading.textContent = 'Legend';
    heading.style.marginTop = '0';
    aside.appendChild(heading);

    const ul = document.createElement('ul');
    ul.style.listStyle = 'none';
    ul.style.padding = '0';
    ul.style.margin = '0';

    entries.forEach(entry => {
        const li = document.createElement('li');
        li.style.marginBottom = '0.5em';

        const colorBox = document.createElement('span');
        colorBox.className = 'legend-entry-color';
        colorBox.style.background = entry.color;

        const label = document.createElement('span');
        label.textContent = entry.label;
        label.className = 'legend-entry-label';

        li.appendChild(colorBox);
        li.appendChild(label);
        ul.appendChild(li);
    });

    aside.appendChild(ul);
    return aside;
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
