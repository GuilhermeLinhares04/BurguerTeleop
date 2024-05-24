var socket = io();

function sendControl(action) {
    socket.emit('control_robot', {action: action});
}

document.getElementById('forward').onclick = function() { sendControl('forward'); };
document.getElementById('backward').onclick = function() { sendControl('backward'); };
document.getElementById('left').onclick = function() { sendControl('left'); };
document.getElementById('right').onclick = function() { sendControl('right'); };
document.getElementById('stop').onclick = function() { sendControl('stop'); };
document.getElementById('emergency_stop').onclick = function() { sendControl('emergency_stop'); };
