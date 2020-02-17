//Gear 2 Swipe Gesture Tutorial
//----------------------------------

//Copyright (c)2014 Dibia Victor, Denvycom
//Modified by Hyunjun
//Distributed under MIT license

//https://github.com/chuvidi2003/AcceleroMeterGear
$(window).load(function(){

	var onSave = false, docuDir, newFile, fileStream, startTime, SService, UVSensor, LSensor, MagSensor, PSensor, HAM, data2Write = [], bgBtn, saveBtn, endSaveBtn;
	
	//for GPS
	var loc = navigator.geolocation;
	
	//end save. close stream
	function offSave(){
		console.log("Save off");
		onSave = false;
		fileStream.close();
		document.getElementById("isSave").innerHTML = "Save OFF";
	}
	
	//This listens for the back button press
	document.addEventListener('tizenhwkey', function(e) {
		if(e.keyName === "back"){
			tizen.application.getCurrentApplication().exit();
			offSave();
		}
	});
	
	//callback if it succeeds to start sensor (except Heart Rate, GPS)
	function onStartSensor(sensor){
		console.log("Sensor" + sensor + "start");
	}
	
	//callback if it fails (sensor start, getData)
	function onFailSensor(error){
		console.log("Fail : " + error);
	}

	//for name of file
	startTime = tizen.time.getCurrentDateTime();

	//time to String
	function timePrint(time){
		var hour, minute, second, milSec;
		
		hour = time.getHours();
		minute = time.getMinutes();
		second = time.getSeconds();
		milSec = time.getMilliseconds();
		
		return hour.toString()+"_" + minute.toString() + "_" + second.toString() + "_" + milSec.toString();
	}
	
	//create txt file
	tizen.filesystem.resolve("documents", 
		function(result){
			docuDir = result;
			newFile = docuDir.createFile(startTime.getMonth().toString() + "_" +startTime.getDate().toString()+"_"+timePrint(startTime)+".txt");
			console.log("file open : " + newFile);
		}, function(error){
			console.log("error : " + error.message);
		}, 'rw'
	);
	
	//for convenience
	document.getElementById("time").innerHTML = "Start at " + timePrint(startTime);
	
	//Sensor service is require to start each sensor
	SService = window.webapis&&window.webapis.sensorservice;	//tizen.sensorservice is not worked
	UVSensor = SService.getDefaultSensor("ULTRAVIOLET");
	LSensor = SService.getDefaultSensor("LIGHT");
	MagSensor = SService.getDefaultSensor("MAGNETIC");
	PSensor = SService.getDefaultSensor("PRESSURE");
	HAM = (tizen&&tizen.humanactivitymonitor)||(window.webapis&&window.webapis.motion);	//Heart Rate is another application, not sensor service
	
	//start each sensor
	UVSensor.start(onStartSensor, onFailSensor);
	LSensor.start(onStartSensor, onFailSensor);
	MagSensor.start(onStartSensor, onFailSensor);
	PSensor.start(onStartSensor, onFailSensor);
	
	//add listener. listener is called when sensor value has changed
	UVSensor.setChangeListener(function(data){
		document.getElementById("UV").innerHTML = 'UV : ' + data.ultravioletLevel;
			
		data2Write[10] = data.ultravioletLevel;
	});
	
	MagSensor.setChangeListener(function(data){
		document.getElementById("xmag").innerHTML = 'Mag X : ' + data.x;
		document.getElementById("ymag").innerHTML = 'Mag Y : ' + data.y;
		document.getElementById("zmag").innerHTML = 'Mag Z : ' + data.z;
		
		data2Write[7] = data.x;
		data2Write[8] = data.y;
		data2Write[9] = data.z;
	});
	
	LSensor.setChangeListener(function(data){
		document.getElementById("light").innerHTML = 'Light : ' + data.lightLevel;
		
		data2Write[11] = data.lightLevel;
	});
	
	PSensor.setChangeListener(function(data){
		document.getElementById("press").innerHTML = 'Pressure : ' + data.pressure;
		
		data2Write[12] = data.pressure;
	});

	//start Heart Rate sensor
	HAM.start("HRM",function(data){
		document.getElementById("heart").innerHTML = 'HeartRate : ' + data.heartRate;
		
		data2Write[13] = data.heartRate;
	});
	
	//start GPS
	HAM.start("GPS",function(data){
		var GPSdata = data.gpsInfo[0];
		
		document.getElementById("latitude").innerHTML = 'Latitude : ' + GPSdata.latitude;
		document.getElementById("longitude").innerHTML = 'Longitude : ' + GPSdata.longitude;
		document.getElementById("altitude").innerHTML = 'Altitude : ' + GPSdata.altitude;
		document.getElementById("speed").innerHTML = 'Speed : ' + GPSdata.speed;
		
		data2Write[14] = GPSdata.latitude;
		data2Write[15] = GPSdata.longitude;
		data2Write[16] = GPSdata.altitude;
		data2Write[17] = GPSdata.speed;
	});
	
	//get acceleration and angular velocity (reference : Mozilla Web API)
	window.addEventListener('devicemotion', function(e) {
		var ax,ay,az,rotx,roty,rotz,interval,currTime;
		
		ax = e.accelerationIncludingGravity.x;
		ay = -e.accelerationIncludingGravity.y;
		az = -e.accelerationIncludingGravity.z;
		rotx = e.rotationRate.alpha ;
		roty = e.rotationRate.beta ;
		rotz = e.rotationRate.gamma ;
		interval = e.interval;
		
		data2Write[0] = interval;
		data2Write[1] = ax;
		data2Write[2] = ay;
		data2Write[3] = az;
		data2Write[4] = rotx;
		data2Write[5] = roty;
		data2Write[6] = rotz;

		document.getElementById("interv").innerHTML =  'Interval : ' +  interval;
		document.getElementById("xaccel").innerHTML =  'AccX : ' +  ax;
		document.getElementById("yaccel").innerHTML = 'AccY : ' + ay;
		document.getElementById("zaccel").innerHTML = 'AccZ : ' + az;
		
		document.getElementById("rotx").innerHTML = 'Rot X : ' + rotx ;
		document.getElementById("roty").innerHTML = 'Rot Y : ' + roty ;
		document.getElementById("rotz").innerHTML = 'Rot Z : ' + rotz ;
		
		/*loc.getCurrentPosition(function(data){
			var GPSdata = data.data.coords;
			
			document.getElementById("latitude").innerHTML = 'Latitude : ' + GPSdata.latitude;
			document.getElementById("longitude").innerHTML = 'Longitude : ' + GPSdata.longitude;
			document.getElementById("altitude").innerHTML = 'Altitude : ' + GPSdata.altitude;
			document.getElementById("speed").innerHTML = 'Speed : ' + GPSdata.speed;
			
			data2Write[14] = GPSdata.latitude;
			data2Write[15] = GPSdata.longitude;
			data2Write[16] = GPSdata.altitude;
			data2Write[17] = GPSdata.speed;
		});*/
		
		//save all data to txt
		if(onSave === true){
			currTime = tizen.time.getCurrentDateTime();
			fileStream.write(timePrint(currTime)+","+data2Write.toString()+"\n");
			console.log(data2Write.toString());
		}
	});
	
	//app hide. Need to enable background-support in config.xml
	bgBtn = document.getElementById("bg-btn");
	bgBtn.addEventListener("click", function(){
		var app = tizen.application.getCurrentApplication();
		app.hide();
	});//add event listener to app hide button
	
	//start save. open stream to save
	saveBtn = document.getElementById("save-btn");
	saveBtn.addEventListener("click", function(){
		console.log("Save on");
		newFile.openStream("w",
			function(fs){
				fileStream = fs;
				console.log("Hello gear!");
			}, function(e){
				console.log("error : " + e.message);
			});
		onSave = true;
		document.getElementById("isSave").innerHTML = "Save ON";
	});//add event listener to save button
	
	endSaveBtn = document.getElementById("saveEnd-btn");
	endSaveBtn.addEventListener("click", offSave);//add event listener to save end button
	
});