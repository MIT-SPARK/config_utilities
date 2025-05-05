function buildSelectionPane(available, selected, src_id) {
    var src = document.getElementById(src_id);
    var html = src.innerHTML;

    if (available.length == 0) {
        html += "Currently no options are available...";
        src.innerHTML = html;
        return;
    }

    // TODO(lschmid): Consider flexible layouts if many elements are available in the future.
    for (var i = 0; i < available.length; i++) {
        var d = available[i];
        html += "<button class='config-button"
        if (d == selected) {
            html += " button-selected";
        }
        html += "' type='submit'";
        if (d != selected) {
            html += " onclick=\"post('/select', {'" + src_id + "': '" + d + "'})\"";
        }
        html += ">" + d + "</button>";
    }
    src.innerHTML = html;
}
