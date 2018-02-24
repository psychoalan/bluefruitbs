// Based on an example:
//https://github.com/don/cordova-plugin-ble-central


// ASCII only
function bytesToString(buffer) {
    return String.fromCharCode.apply(null, new Uint8Array(buffer));
}

// ASCII only
function stringToBytes(string) {
    var array = new Uint8Array(string.length);
    for (var i = 0, l = string.length; i < l; i++) {
        array[i] = string.charCodeAt(i);
    }
    return array.buffer;
}

// this is ble hm-10 UART service
/*var blue= {
    serviceUUID: "0000FFE0-0000-1000-8000-00805F9B34FB",
    characteristicUUID: "0000FFE1-0000-1000-8000-00805F9B34FB"
};*/

//the bluefruit UART Service
var blue ={
	serviceUUID: '6e400001-b5a3-f393-e0a9-e50e24dcca9e',
    txCharacteristic: '6e400002-b5a3-f393-e0a9-e50e24dcca9e', // transmit is from the phone's perspective
    rxCharacteristic: '6e400003-b5a3-f393-e0a9-e50e24dcca9e'  // receive is from the phone's perspective
}

var ConnDeviceId;
var deviceList =[];
 
function onLoad(){
	document.addEventListener('deviceready', onDeviceReady, false);
    bleDeviceList.addEventListener('touchstart', conn, false); // assume not scrolling
}

function onDeviceReady(){
	refreshDeviceList();
}

	 
function refreshDeviceList(){
	//deviceList =[];
	document.getElementById("bleDeviceList").innerHTML = ''; // empties the list
	if (cordova.platformId === 'android') { // Android filtering is broken
		ble.scan([], 5, onDiscoverDevice, onError);
	} else {
		//alert("Disconnected");
		ble.scan([blue.serviceUUID], 5, onDiscoverDevice, onError);
	}
}


function onDiscoverDevice(device){
	//Make a list in html and show devises
	var listItem = document.createElement('li'),
    html = device.name+ "," + device.id;
    listItem.innerHTML = html;
    document.getElementById("bleDeviceList").appendChild(listItem);
}


function conn(){
	
	var  deviceTouch= event.srcElement.innerHTML;
	document.getElementById("debugDiv").innerHTML =""; // empty debugDiv
	var deviceTouchArr = deviceTouch.split(",");
	ConnDeviceId = deviceTouchArr[1];
	//for debug:
	document.getElementById("debugDiv").innerHTML += "<br>"+deviceTouchArr[0]+"<br>"+deviceTouchArr[1];
	ble.connect(ConnDeviceId, onConnect, onConnError);
 }
 
function onConnect(){
	document.getElementById("statusDiv").innerHTML = " Status: Connected";
	document.getElementById("bleId").innerHTML = ConnDeviceId;
	ble.startNotification(ConnDeviceId, blue.serviceUUID, blue.rxCharacteristic, onData, onError);
	 // ble.startNotification(deviceId, bluefruit.serviceUUID, bluefruit.rxCharacteristic, app.onData, app.onError);
}

function onConnError(){
	alert("Problem connecting");
	document.getElementById("statusDiv").innerHTML = " Status: Disonnected";
}

 function onData(data){ // data received from Arduino
	document.getElementById("receiveDiv").innerHTML =  "Received: " + bytesToString(data) + "<br/>";
}

//should be deleted
function data(txt){
	messageInput.value = txt;
}	

function sendData(dataStr) { // send data to Arduino
	 var data = stringToBytes(dataStr);
	ble.writeWithoutResponse(ConnDeviceId, blue.serviceUUID, blue.txCharacteristic, data, onSend, onError);
}
	
function onSend(){
	document.getElementById("sendDiv").innerHTML = "Sent: " + messageInput.value + "<br/>";
}

function disconnect() {
	ble.disconnect(deviceId, onDisconnect, onError);
}

function onDisconnect(){
	document.getElementById("statusDiv").innerHTML = "Status: Disconnected";
}
function onError(reason)  {
	alert("ERROR: " + reason); // real apps should use notification.alert
}
function myFunction() {
	var div = document.createElement('div');
	div.innerHTML = "Alarm set to: " + document.getElementById("alarmHour").value + ":" + " " + document.getElementById("alarmMinute").value
	+ ":"+" "+document.getElementById("alarmSecond").value;
	document.body.appendChild(div);
	
	
	var today = new Date();
	var h = today.getHours();
	var m = today.getMinutes();
	var s = today.getSeconds();
	
	var div = document.createElement('div');
	
	div.innerHTML = "Current Time: " + h + ":" + m +":"+ s;
	document.body.appendChild(div);
	
	
	var setTimeInSeconds= +document.getElementById("alarmHour").value * +3600 + +document.getElementById("alarmMinute").value*+60+ +document.getElementById("alarmSecond").value;
	var currentTimeInSecond = parseInt(+h*+3600 + +m*+60+ +s);
	
	console.log("Alarm set" + setTimeInSeconds);
	console.log("Current time" + currentTimeInSecond);
	
	
	
	if(setTimeInSeconds>currentTimeInSecond){
		var alarmRings1= setTimeInSeconds - currentTimeInSecond;
		var alarmRings = alarmRings1.toString(); 
		var div = document.createElement('div');
		
		div.innerHTML = "Alarm will ring in: " + alarmRings +"seconds" ;
		document.body.appendChild(div);
		sendTime(alarmRings);
	}
	else{
		var alarmRings1= setTimeInSeconds + (86400-currentTimeInSecond);
		var alarmRings = alarmRings1.toString(); 
		var div = document.createElement('div');
		div.innerHTML = "Alarm will ring in: " + alarmRings + " seconds";
		document.body.appendChild(div);
		sendTime(alarmRings);
	}
}
function sendTime(alarmRingsStr) { // send alarm to Arduino
	var alarmRings = stringToBytes(alarmRingsStr);
	ble.writeWithoutResponse(ConnDeviceId, blue.serviceUUID, blue.txCharacteristic, alarmRings, onSend, onError);
}