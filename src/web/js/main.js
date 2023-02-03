$.ajaxSetup({
    async: false
});
var currentURL = window.location.href;
function controlCoreApp(checkbox){
    var action;
    if (checkbox.checked)
    {
        action = "on";
    }
    else
    {
        action = 'off'
    }
    $.ajax({
        url: currentURL+'/control',
        type: 'POST',
        data: jQuery.param({'action': action}),
        contentType: 'application/x-www-form-urlencoded; charset=UTF-8',
        async: false,
        success: function(result) {}
    });
}
// Play audio
//function playWarning() {
//    let warning = new Audio(window.location.href+'/warning');
//    warning.play();
////    document.getElementById("warning").play();
//}
//
//// Run a callback function once every second
let timer = setInterval(function () {
        clearInterval(timer);
        reload();
 }, 1000);

function reload(){
    document.getElementById('warn').src = currentURL+'/warning';
}