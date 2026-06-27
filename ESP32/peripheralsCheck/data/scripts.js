let websocket, pingTimeout, connectTimeout;
let gateway = ("ws://") + window.location.host + "/webserialws";
let textArea = document.getElementById("record");

function initWebPage() {
    document.getElementById("command-text").disabled = true;
    initWebSocket();
    document.getElementById("command-text").addEventListener("input", sendCommand);
}

function initWebSocket() {
    clearTimeout(connectTimeout);
    clearTimeout(pingTimeout);
    pingTimeout = false;
    connectTimeout = setTimeout((() => {
        terminalWrite("[WebSerial] Connect timeout."),
            websocket.close(),
            initWebSocket()
    }
    ), 3e3);
    terminalWrite("[WebSerial] Connecting ...");
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
    websocket.onerror = onError;
}

function onOpen(e) {
    clearTimeout(connectTimeout);
    terminalWrite("[WebSerial] Connected successfully.");
    document.getElementById("command-text").disabled = false;
}

function onError(e) {
    console.log("[WebSerial] Error!", e);
    websocket.close();
}

function onClose(e) {
    console.log("[WebSerial] Connection closed.", e)
}
function onMessage(e) {
    "pong" == e.data ? (clearTimeout(pingTimeout),
        pingTimeout = false) : terminalWrite("received: " + e.data);
}

function terminalWrite(e) {
    textArea.value += e + "\n";
    //textArea.scrollTop = textArea.scrollHeight;
}

function terminalClean() {
    textArea.value = "";
    textArea.scrollTop = textArea.scrollHeight;
}

function sendCommand() {
    let e = document.getElementById("command-text").value;
    console.log("send command: ", e);
    terminalWrite("send: " + e);
    websocket.send(e);
    document.getElementById("command-text").value = "";
}

setInterval((() => {
    pingTimeout || websocket.readyState != WebSocket.OPEN || (pingTimeout = setTimeout((() => {
        terminalWrite("[WebSerial] Ping timeout.");
        websocket.close();
        initWebSocket();
    }
    ), 3e3),
        websocket.send("ping"))
}
), 2e3);

window.addEventListener("DOMContentLoaded", (function () {
    initWebPage()
}
), false)