//Gear 2 Swipe Gesture Tutorial
//----------------------------------

//Copyright (c)2014 Dibia Victor, Denvycom
//Modified by Hyunjun
//Distributed under MIT license

//https://github.com/chuvidi2003/AcceleroMeterGear


	//var webSocketUrl = "ws://192.168.0.5:9090/";
	var webSocketUrl = "ws://172.20.10.4:9090/"; //8-4
	var webSocket;
	webSocket = new WebSocket(webSocketUrl);

	// If the connection is established 
	 
	webSocket.onopen = function(e) 
	{
	   console.log('connection open, readyState: ' + e.target.readyState);
	};

	 //If the connection fails or is closed with prejudice 
	webSocket.onerror = function(e) 
	{
	   /* Error handling */
		console.log('connection closed, readyState: ' + e.target.readyState);
	};
	


$(window).load(function(){
	
	
	var onSave = false, docuDir, newFile, fileStream, startTime, SService, UVSensor, LSensor, MagSensor, PSensor, HAM, data2Write = [],mode0Btn,mode1Btn,mode2Btn, bgBtn, saveBtn, endSaveBtn;
	
	var pitch=0, roll=0, yaw=0;
	
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
		document.getElementById("xmag").innerHTML = 'Mag X : ' + -data.x;
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
	 
	var currTime = tizen.time.getCurrentDateTime();
	
	window.addEventListener('deviceorientation', function(e){
		data2Write[18] = -e.beta;
		data2Write[19] = e.gamma;
		data2Write[20] = e.alpha;
		
		document.getElementById("pitch").innerHTML = 'Pitch : ' + (e.gamma)  ;
		document.getElementById("roll").innerHTML = 'Roll : ' + (-e.beta) ;
		document.getElementById("yaw").innerHTML = 'Yaw : ' + (e.alpha) ;
	},true);
	
	
	var ax_=0, ay_=0, az_=0, mode=0,ax,ay,az;
	//get acceleration and angular velocity (reference : Mozilla Web API)
	window.addEventListener('devicemotion', function(e) {
		var axo,ayo,azo, axi,ayi,azi,rotxi,rotyi,rotzi,rotx,roty,rotz,interval, magyaw;

		axi = -e.accelerationIncludingGravity.x;
		ayi = e.accelerationIncludingGravity.y; //- attach (-)
		azi = e.accelerationIncludingGravity.z;
		rotxi =  e.rotationRate.alpha ;
		rotyi = -e.rotationRate.beta ;
		rotzi = -e.rotationRate.gamma ;
		
		//apply calibration
		ax= (9005086496992679*axi)/9007199254740992 - (970847136413691*ayi)/1152921504606846976 + (4676546001099593*azi)/4611686018427387904 - 8343091890828650998003174283065555/20769187434139310514121985316880384;
		ay= (9003026989503915*ayi)/9007199254740992 - (4684841617841737*azi)/2305843009213693952 - 1273430245934866820054511086505921/2596148429267413814265248164610048;
		az= (2252626011982959*azi)/2251799813685248 + 2563386722619340963857975513927/2535301200456458802993406410752;
	
		
		axo = (-Math.sin(data2Write[20]*Math.PI/180)*Math.sin(data2Write[18]*Math.PI/180)*Math.sin(data2Write[19]*Math.PI/180)+Math.cos(data2Write[20]*Math.PI/180)*Math.cos(data2Write[19]*Math.PI/180))*ax + (-Math.sin(data2Write[20]*Math.PI/180)

*Math.cos(data2Write[18]*Math.PI/180))*ay + (Math.sin(data2Write[20]*Math.PI/180)*Math.sin(data2Write[18]*Math.PI/180)*Math.cos(data2Write[19]*Math.PI/180)+Math.cos(data2Write[20]*Math.PI/180)*Math.sin(data2Write[19]*Math.PI/180))*az;
		ayo = (Math.cos(data2Write[20]*Math.PI/180)*Math.sin(data2Write[18]*Math.PI/180)*Math.sin(data2Write[19]*Math.PI/180)+Math.sin(data2Write[20]*Math.PI/180)*Math.cos(data2Write[19]*Math.PI/180))*ax + (Math.cos(data2Write[20]*Math.PI/180)

*Math.cos(data2Write[18]*Math.PI/180))*ay + (-Math.cos(data2Write[20]*Math.PI/180)*Math.sin(data2Write[18]*Math.PI/180)*Math.cos(data2Write[19]*Math.PI/180)+Math.sin(data2Write[20]*Math.PI/180)*Math.sin(data2Write[19]*Math.PI/180))*az;
		azo = (-Math.cos(data2Write[18]*Math.PI/180)*Math.sin(data2Write[19]*Math.PI/180))*ax + (Math.sin(data2Write[18]*Math.PI/180))*ay + (Math.cos(data2Write[18]*Math.PI/180)*Math.cos(data2Write[19]*Math.PI/180))*az;
		
		ax = axo-ax_;
		ay = ayo-ay_;
		az = azo-az_;
		
		
		ax_=axo;
		ay_=ayo;
		az_=azo;
		
		
		ax=parseInt(axo*100)/100;
		ay=parseInt(ayo*100)/100;
		az=parseInt(azo*100)/100;
		
		rotx= (4532074390415255*rotxi)/4503599627370496 + (2883896438722165*rotyi)/72057594037927936 - (8721539557351861*rotzi)/288230376151711744;
		roty=  (2432704819743611*rotxi)/72057594037927936 + (554278758788703*rotyi)/562949953421312 + (7048490289617383*rotzi)/72057594037927936;
		rotz=  (4157351154835707*rotzi)/4503599627370496 - (3328889871184437*rotyi)/144115188075855872 - (5800857746681135*rotxi)/1152921504606846976;
				
		var tmp = tizen.time.getCurrentDateTime();
		
		if(tmp.getSeconds()==currTime.getSeconds()){
		interval = tmp.getMilliseconds() - currTime.getMilliseconds();
		}else{
			interval = 1000 + tmp.getMilliseconds() - currTime.getMilliseconds();
		}
		currTime= tmp;
	
		magyaw = Math.atan2(data2Write[8]*Math.cos(roll*Math.PI/180)-data2Write[9]*Math.sin(roll*Math.PI/180),
			data2Write[7]*Math.cos(pitch*Math.PI/180)+data2Write[8]*Math.sin(pitch*Math.PI/180)*Math.sin(roll*Math.PI/180)+data2Write[9]*Math.cos(roll*Math.PI/180)*Math.sin(pitch*Math.PI/180));
	
		yaw = 0.95*(yaw+rotz*interval/1000)+0.05*magyaw*180/Math.PI;
		

		data2Write[0] = interval;
		data2Write[1] = ax;
		data2Write[2] = ay;
		data2Write[3] = az;
		data2Write[4] = rotx;
		data2Write[5] = roty;
		data2Write[6] = rotz;

		roll = data2Write[18];
		pitch = data2Write[19];
		yaw = data2Write[20];
		
		document.getElementById("interv").innerHTML =  'Interval : ' +  interval;
		document.getElementById("xaccel").innerHTML =  'AccX : ' +  ax;
		document.getElementById("yaccel").innerHTML = 'AccY : ' + ay;
		document.getElementById("zaccel").innerHTML = 'AccZ : ' + az;
		
		document.getElementById("rotx").innerHTML = 'Rot X : ' + rotx ;
		document.getElementById("roty").innerHTML = 'Rot Y : ' + roty ;
		document.getElementById("rotz").innerHTML = 'Rot Z : ' + rotz ;

		//save all data to txt
		if(onSave === true){
			fileStream.write(timePrint(currTime)+","+data2Write.toString()+"\n");
			console.log(timePrint(currTime)+","+data2Write.toString());
		}
		if(mode === 0){
			var msg = {"msg": {"data": 0+"^"+0+"^"+0+"^"+0},
					 "op": "publish",
					 "topic": "/pThrust"
					};
			webSocket.send(JSON.stringify(msg));
		}
		if(mode === 1){
			roll=parseInt(roll*100)/100;
			pitch=parseInt(pitch*100)/100;
			yaw=parseInt(yaw*100)/100;
			var msg = {"msg": {"data": mode+"^"+roll+"^"+pitch+"^"+yaw},
					 "op": "publish",
					 "topic": "/pThrust"
					};
			webSocket.send(JSON.stringify(msg));
		}
		if(mode === 2){
			var msg = {"msg": {"data": mode+"^"+ax+"^"+ay+"^"+az},
					 "op": "publish",
					 "topic": "/pThrust"
					};
			webSocket.send(JSON.stringify(msg));
			
		}
		
	});
	
	mode0Btn = document.getElementById("mode0-btn");
	mode0Btn.addEventListener("click",function(){
		mode=0;
	});
	
	mode1Btn = document.getElementById("mode1-btn");
	mode1Btn.addEventListener("click",function(){
		mode =1;

	});
	
	mode2Btn = document.getElementById("mode2-btn");
	mode2Btn.addEventListener("click",function(){
		mode=2;
	
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
	
} );