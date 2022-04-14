// This file is required by the index.html file and will
// be executed in the renderer process for that window.
// No Node.js APIs are available in this process because
// `nodeIntegration` is turned off. Use `preload.js` to
// selectively enable features needed in the rendering
// process.


const { SerialPort } = require('serialport')
const tableify = require('tableify')

async function listSerialPorts() {
  await SerialPort.list().then((ports, err) => {
    if(err) {
      document.getElementById('error').textContent = err.message
      return
    } else {
      document.getElementById('error').textContent = ''
    }
    console.log('ports', ports);

    if (ports.length === 0) {
      document.getElementById('error').textContent = 'No ports discovered'
    }

    document.getElementById('ports').innerHTML = "";

    for (i = 0; i < ports.length; i++){
        let btn = document.createElement("button");

        let path = ports[i].path
        btn.name = path;
        btn.innerHTML = path;
        btn.onclick = function () {
            console.log(path);
            var sp = new SerialPort(path, { //pi
                baudRate: 115200,
                autoOpen: true,
            },function (err) {
                if (err) {
                    return console.log('Error: ', err.message)
                }
            });
            alert("opening port");
        };
        document.getElementById('ports').appendChild(btn);
    }

    // tableHTML = tableify(ports)
  })
}

function listPorts() {
  listSerialPorts();
  setTimeout(listPorts, 2000);
}

// Set a timeout that will check for new serialPorts every 2 seconds.
// This timeout reschedules itself.
setTimeout(listPorts, 2000);

listSerialPorts()
