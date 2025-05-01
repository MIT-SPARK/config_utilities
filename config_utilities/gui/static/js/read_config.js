function readConfig() {
    // const data = document.getElementById("config-table").innerHTML
    // $.ajax({
    //     type: "POST",
    //     // url: '{{ url_for('view.path') }}'
    //     url: "/", //"/postmethod",
    //     data: { val: "asdasdasd" }
    // });
    $.post("/getconfig", {
        javascript_data: "asd"
    });
}