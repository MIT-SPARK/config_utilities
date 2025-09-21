
function setupSummaryView() {
    html = "<h2>Summary View</h2>";
    html += "<p>This view summarizes the changes made to the configuration over time.</p>";
    html += `
        <div style="
            background: #fff;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.08);
            padding: 1em;
            margin-top: 1em;
            box-sizing: border-box;
            width: 100%;
            height: 100%;
            overflow: auto;
            display: flex;
            flex-direction: row;
            gap: 2em;
        ">
            <div id="summary-panel-content" style="flex: 1 1 0;">
                <!-- Summary content will be displayed here -->
            </div>
            <aside style="
                flex: 0 0 25%;
                max-width: 25%;
                min-width: 180px;
                box-sizing: border-box;
                padding-left: 1em;
                border-left: 1px solid #eee;
                display: flex;
                flex-direction: column;
                justify-content: flex-start;
            ">
                <h3 style="margin-top:0;">Legend</h3>
                <ul style="list-style: none; padding: 0; margin: 0;">
                    <li style="margin-bottom: 0.5em;">
                        <span style="display:inline-block;width:16px;height:16px;background:#4caf50;border-radius:3px;margin-right:8px;vertical-align:middle;"></span>
                        <span style="vertical-align:middle;">Added</span>
                    </li>
                    <li style="margin-bottom: 0.5em;">
                        <span style="display:inline-block;width:16px;height:16px;background:#ff9800;border-radius:3px;margin-right:8px;vertical-align:middle;"></span>
                        <span style="vertical-align:middle;">Modified</span>
                    </li>
                    <li>
                        <span style="display:inline-block;width:16px;height:16px;background:#f44336;border-radius:3px;margin-right:8px;vertical-align:middle;"></span>
                        <span style="vertical-align:middle;">Removed</span>
                    </li>
                </ul>
            </aside>
        </div>
    `;
    document.getElementById("displayPane").innerHTML = html;
    document.getElementById("summary-panel-content").appendChild(renderYamlLike(window.introspectionData, "#4caf50"));
}


