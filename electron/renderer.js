// This file is required by the index.html file and will
// be executed in the renderer process for that window.
// No Node.js APIs are available in this process because
// `nodeIntegration` is turned off. Use `preload.js` to
// selectively enable features needed in the rendering
// process.

// const Readline = serialPort.parsers.Readline;
const { ReadlineParser } = require('@serialport/parser-readline')        
const { SerialPort } = require('serialport')
// const tableify = require('tableify')


var platform = process.platform;
console.log(platform);

var portPath 

if (platform == "linux"){
  portPath = '/dev/ttyACM0'; //PI
}else{
  portPath = '/dev/tty.usbmodem101457901'; //MBP
}
// var portPath = '/dev/tty.usbmodem101457901'; //MBP
 //Pi

//initialize serialport with 9600 baudrate.
// var sp = new SerialPort('/dev/tty.usbmodem142301', {
var sp = new SerialPort({path: portPath, // MBP
// var sp = new SerialPort({path: '/dev/ttyACM0', //pi
    baudRate: 115200,
    autoOpen: true,
},function (err) {
    if (err) {
        return console.log('Error: ', err.message)
    }
});



//parse incoming data line-by-line from serial port.
// const parser = sp.pipe(new Readline({ delimiter: '\r\n' }));
const parser = sp.pipe(new ReadlineParser({ delimiter: '\r\n' }));
parser.on('data', processSerialData);

sp.on('error', function(err) {
    console.log('Error: ', err.message)
})

let PIDsp = 0.0;
let gram = 0.0;
let seconds = 0.0;

//append incoming data to the textarea.
function processSerialData(event) {
    if (event.startsWith("{")){
        document.getElementById("serialJSON").value = event;
        const data = JSON.parse(event);
        console.log(data);
        document.getElementById("grams").innerHTML = data.g + "gram";
        document.getElementById("seconds").innerHTML = data.s + "s";
        document.getElementById("temp").innerHTML = data.t + "\u00B0C";
        document.getElementById("setpoint").innerHTML = data.PIDsp + "\u00B0C";
        PIDsp = data.PIDsp;
        gram = data.g;
        seconds = data.s;
    }else{
        document.getElementById("incomingData").value += "\n"+event;
    }
}

function writeonSer(data){
    //Write the data to serial port.
    sp.write( data, function(err) {
        if (err) {
            return console.log('Error on write: ', err.message);
        }
        console.log('message written');
    });

}

function tare(){
  writeonSer('<s:t>');
}

function autoTare(){
  if (seconds < 0.2){
    tare();
  }
}

document.getElementById('inputText').onkeypress = function(e){
    if (!e) e = window.event;
    var keyCode = e.keyCode || e.which;
    //write the data to serial when enter key is pressed.
    if (keyCode == '13'){
        //console.log("enter pressed", document.getElementById("inputText").value);
        writeonSer(document.getElementById("inputText").value+"\r\n");
        document.getElementById('inputText').value = "";
        return false;
    }
}
document.getElementById('tare').onclick = function(e){
    //send ctrl+c to serialport
    tare();
}
document.getElementById('start').onclick = function(e){
    //send ctrl+c to serialport
    writeonSer('<t:s>');
}
document.getElementById('stop').onclick = function(e){
    //send ctrl+c to serialport
    writeonSer('<t:p>');
}
document.getElementById('reset').onclick = function(e){
    //send ctrl+c to serialport
    writeonSer('<t:r>');
}
document.getElementById('brew10').onclick = function(e){
  autoTare();
  writeonSer('<bt:10>');
}
document.getElementById('brew25').onclick = function(e){
  autoTare();
  writeonSer('<bt:25>');
}
document.getElementById('brew26').onclick = function(e){
  autoTare();
  writeonSer('<bt:26>');
}
document.getElementById('brew27').onclick = function(e){
  autoTare();
  writeonSer('<bt:27>');
}
document.getElementById('brew28').onclick = function(e){
  autoTare();
  writeonSer('<bt:28>');
}
document.getElementById('brew29').onclick = function(e){
  autoTare();
  writeonSer('<bt:29>');
}
document.getElementById('brew30').onclick = function(e){
  autoTare();
  writeonSer('<bt:30>');
}
document.getElementById('brewStop').onclick = function(e){
  writeonSer('<bstop:0>');
}
document.getElementById('sp20').onclick = function(e){
  writeonSer('<PIDcSP:20>');
}
document.getElementById('sp85').onclick = function(e){
  writeonSer('<PIDcSP:85>');
}
document.getElementById('sp90').onclick = function(e){
  writeonSer('<PIDcSP:90>');
}
document.getElementById('sp95').onclick = function(e){
  writeonSer('<PIDcSP:95>');
}
document.getElementById('sp110').onclick = function(e){
  writeonSer('<PIDcSP:110>');
}
document.getElementById('sp120').onclick = function(e){
  writeonSer('<PIDcSP:120>');
}

document.getElementById('temp-up').onclick = function(e){
  console.log(PIDsp + 1)
  writeonSer(`<PIDcSP:${PIDsp + 1}>`);
}
document.getElementById('temp-down').onclick = function(e){
  console.log(PIDsp - 1)
  writeonSer(`<PIDcSP:${PIDsp - 1}>`);
}

const { exec } = require("child_process");

document.getElementById('reboot').ondblclick = function(e){
  exec('reboot');
}


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

        var path = ports[i].path
        btn.name = path;
        btn.innerHTML = path;
        portPath = path;
        btn.onclick = function () {
            console.log(path);
            sp = new SerialPort({path:path, //pi
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

function reOpenPort() {
  if (!sp.isOpen){
    console.log('serial port is closed, attempting to reopen');
    sp.open(function (err) {
      if (err) {
        return console.log('Error opening port: ', err.message)
      }
    })
  }
  setTimeout(reOpenPort, 2000);
}

// Set a timeout that will check for new serialPorts every 2 seconds.
// This timeout reschedules itself.
setTimeout(listPorts, 2000);

setTimeout(reOpenPort, 2000);

listSerialPorts()
