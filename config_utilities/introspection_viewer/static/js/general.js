function updateDisplay() {
    // TODO(lschmid): For now just reset the display panel and re-create everything.
    setupDisplayPane();
    const displayPanel = document.getElementById("displayPanel");
    const panelContent = document.getElementById("displayPanelContent");
    if (window.introSettings.current_view === "summary") {
        setupSummaryView(displayPanel, panelContent);
    }
    // Future views can be added here.
}

function summaryBtnClicked(button) {
    if (window.introSettings.current_view !== "summary") {
        window.introSettings.current_view = "summary";
        updateDisplay();
        button.classList.add("button-selected");
    }
}

function optionButtonClicked(button, option) {
    window.introSettings[option] = !window.introSettings[option];
    button.classList.toggle("button-selected");
    updateDisplay();
}

function setupDisplayPane() {
    // Format the display pane
    const displayPane = document.getElementById("displayPane");
    displayPane.innerHTML = "";

    const heading = document.createElement("h2");
    heading.textContent = "Summary View";
    displayPane.appendChild(heading);

    const displayPanel = document.createElement("div");
    displayPanel.className = "display-pane";
    displayPanel.id = "displayPanel";
    displayPanel.style.display = "flex";
    displayPanel.style.gap = "2em";
    displayPanel.style.position = "relative";

    const panelContent = document.createElement("div");
    panelContent.id = "displayPanelContent";
    panelContent.style.maxHeight = `calc(100vh - ${displayPane.getBoundingClientRect().top}px - 10em)`;
    panelContent.style.overflowY = "auto";
    panelContent.style.width = "100%";
    panelContent.style.minWidth = "25%";
    panelContent.style.overflowX = "auto";
    panelContent.style.flexGrow = "1";
    panelContent.style.flexShrink = "1";
    panelContent.style.boxSizing = "border-box";

    displayPanel.appendChild(panelContent);
    displayPane.appendChild(displayPanel);
}
