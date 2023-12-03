document.getElementById("power_input").value = 0;

function updateValue(value) {
    document.getElementById("valueDisplay").textContent = value;
    var xhr = new XMLHttpRequest();
    xhr.open("POST", "/submit", true);
    xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
    xhr.send(value);
}

// Update Terminal
function updateLog(message) {
    var logElement = document.getElementById("log");
    logElement.textContent += message + '\n';
}

// WebSocket Connection
var socket = new WebSocket("ws://" + window.location.hostname + "/log");
socket.onmessage = function(event) {
    var logMessage = event.data;
    updateLog(logMessage);
}