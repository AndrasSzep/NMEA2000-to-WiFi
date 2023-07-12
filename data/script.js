// Complete project details: https://randomnerdtutorials.com/esp32-web-server-websocket-sliders/

var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
window.addEventListener('load', onload);


// Get references to the canvas elements
const tempcanvas = document.querySelector('#tempchart');
const humcanvas = document.querySelector('#humchart');
const prescanvas = document.querySelector('#preschart');
const watercanvas = document.querySelector('#waterchart');

// Create the chart contexts
const tempCtx = tempcanvas.getContext('2d');
const humCtx = humcanvas.getContext('2d');
const presCtx = prescanvas.getContext('2d');
const waterCtx = watercanvas.getContext('2d');

var currentHour = 10; // Replace with the actual current hour value
var lastHour = 8; // Replace with the actual last hour value
var currentMinute = 10; // Replace with the actual current hour value
var lastMinute = 8; // Replace with the actual last hour value

var fileNames = [
  '/temperature.txt',
  '/humidity.txt',
  '/pressure.txt',
  '/water.txt',
];

var chartContexts = [
  tempCtx,
  humCtx,
  presCtx,
  waterCtx,
];


function fetchFile(fileName, chartContext, chart) {
  fetch(fileName)
    .then(function(response) {
      if (response.ok) {
        return response.text();
      }
      throw new Error('Error: ' + response.status);
    })
    .then(function(fileContent) {
      var fileValues = fileContent.split(',').map(Number);

      chart.data.datasets[0].data = fileValues;
      chart.update();

      console.log('Chart updated successfully.');
    })
    .catch(function(error) {
      console.log(error);
    });
}


// Generate a random value between 15 and 25
function getRandomValue(a,b) {
    return Math.random() * a + b;
}

const tempchart = new Chart(tempCtx, {
    type: 'line',
    data: {
        labels: Array.from({ length: 24 }, (_, i) => (i-24).toString()),
        datasets: [
            {
                label: 'Temperature',
                data: Array.from({ length: 24 }, () => getRandomValue(10,15)),
    			backgroundColor: 'rgba(238,196,30,0.50)',
				borderColor: 'rgba(242,255,83,1.00)',
				borderWidth: 1,
				pointRadius: 1,
				fill: true,
				tension: 0.5,
            },
        ],
    },
    options: {
        responsive: true,
		maintainAspectRatio: false,
		plugins: {
			legend: {
				display: false // Set display to false to hide the legend
			}
		},
		scales: {
            y: {
                beginAtZero: false,
            },
        },
    },
});

const humchart = new Chart(humCtx, {
    type: 'line',
    data: {
        labels: Array.from({ length: 24 }, (_, i) => (i-24).toString()),
        datasets: [
            {
                label: 'Humidity',
                data: Array.from({ length: 24 }, () => getRandomValue(20,40)),
    			backgroundColor: 'rgba(132,238,169,0.50)',
				borderColor: 'rgba(52,186,77,0.88)',
				borderWidth: 1,
				pointRadius: 1,
				fill: true,
				tension: 0.5,
			},
        ],
    },
    options: {
        responsive: true,
		maintainAspectRatio: false,
		plugins: {
			legend: {
				display: false // Set display to false to hide the legend
			}
		},
		scales: {
            y: {
                beginAtZero: false,
            },
        },
    },
});

const preschart = new Chart(presCtx, {
    type: 'line',
    data: {
        labels: Array.from({ length: 24 }, (_, i) => (i-24).toString()),
        datasets: [
            {
                label: 'Pressure',
                data: Array.from({ length: 24 }, () => getRandomValue(20,750)),
    			backgroundColor: 'rgba(255,119,62,0.50)',
				borderColor: 'rgba(225,117,140,1.00)',
				borderWidth: 1,
				pointRadius: 1,
				fill: true,
				tension: 0.5,
			},
        ],
    },
    options: {
        responsive: true,
		maintainAspectRatio: false,
		plugins: {
			legend: {
				display: false // Set display to false to hide the legend
			}
		},
		scales: {
            y: {
                beginAtZero: false,
            },
        },
    },
});

const waterchart = new Chart(waterCtx, {
    type: 'line',
    data: {
        labels: Array.from({ length: 24 }, (_, i) => (i-24).toString()),
        datasets: [
            {
                label: 'Water',
                data: Array.from({ length: 24 }, () => getRandomValue(10,15)),
    			backgroundColor: 'rgba(110,169,228,0.50)',
				borderColor: 'rgba(84,102,255,1.00)',
				borderWidth: 1,
				pointRadius: 1,
				fill: true,
				tension: 0.5,
            },
        ],
    },
    options: {
        responsive: true,
		maintainAspectRatio: false,
		plugins: {
			legend: {
				display: false // Set display to false to hide the legend
			}
		},
		scales: {
            y: {
                beginAtZero: false,
            },
        },
    },
});

var charts = [
  tempchart,
  humchart,
  preschart,
  waterchart
];

fileNames.forEach(function(fileName, index) {
  var chart = charts[index];
  var chartContext = chartContexts[index];

  fetchFile(fileName, chartContext, chart);
});

function onload(event) {
    initWebSocket();
}

function getValues(){
    websocket.send("getValues");
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {
    console.log('Connection opened');
    getValues();
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}
/*
function updateSliderPWM(element) {
    var sliderNumber = element.id.charAt(element.id.length-1);
    var sliderValue = document.getElementById(element.id).value;
    document.getElementById("sliderValue"+sliderNumber).innerHTML = sliderValue;
    console.log(sliderValue);
    websocket.send(sliderNumber+"s"+sliderValue.toString());
}
*/
// Event handler for receiving messages from the server
function onMessage(event) {
 //   console.log(event.data);
	var data = JSON.parse(event.data);
	console.log(data);
    if ('chipid' in data) { document.getElementById('chipid').textContent = data.chipid; }
	if('timedate' in data){
		document.getElementById('timedate').textContent = data.timedate;
		if (data.timedate !== undefined && data.timedate !== null) {
			const timeParts = data.timedate.split(" ");
			if (timeParts.length >= 2) {
				const hourParts = timeParts[1].split(":");
				if (hourParts.length >= 2) {
					currentHour = parseInt(hourParts[0]);
					currentMinute = parseInt(hourParts[1]);
				} else {
					console.log("Invalid time format: missing minute");
				}
			} else if (timeParts.length === 1) {
				const hourParts = timeParts[0].split(":");
				if (hourParts.length >= 2) {
					currentHour = parseInt(hourParts[0]);
					currentMinute = parseInt(hourParts[1]);
				} else {
					console.log("Invalid time format: missing minute");
				}
			} else {
				console.log("Invalid time format");
			}
		} else {
			console.log("timedate is undefined or null");
		}
	}
    // Update the respective spans with the received dat
    if('rpm' in data) {document.getElementById('rpm').textContent = data.rpm;}
    if('depth' in data) {document.getElementById('depth').textContent = data.depth;}
    if('speed' in data) {document.getElementById('speed').textContent = data.speed;}
    if('heading' in data) {document.getElementById('heading').textContent = data.heading;}
    if('windspeed' in data) {document.getElementById('windspeed').textContent = data.windspeed;}
    if('winddir' in data) {document.getElementById('winddir').textContent = data.winddir;}
    if('latitude' in data) {document.getElementById('latitude').textContent = data.latitude;}
    if('longitude' in data) {document.getElementById('longitude').textContent = data.longitude;}
	
//	if (currentMinute !== lastMinute) {
  //		lastMinute = currentMinute;	
			
	console.log("updating charts");
		if( 'airtemp' in data) {
			document.getElementById('airtemp').textContent = data.airtemp;
			var airTemperature = parseInt(data.airtemp, 10); 
    		charts[0].data.datasets[0].data.shift();
    		charts[0].data.datasets[0].data.push(airTemperature);
    		charts[0].update();		
		}
		if( 'humidity' in data) {			
      		document.getElementById('humidity').textContent = data.humidity;
			var airHumidity = parseInt(data.humidity, 10); 
    		charts[1].data.datasets[0].data.shift();
    		charts[1].data.datasets[0].data.push(airHumidity);
    		charts[1].update();
		}
		if( 'pressure' in data) {
	  		document.getElementById('pressure').textContent = data.pressure;
			var airPressure = parseInt(data.pressure, 10); 
    		charts[2].data.datasets[0].data.shift();
    		charts[2].data.datasets[0].data.push(airPressure);
    		charts[2].update();
		}
		if( 'watertemp' in data) {
			document.getElementById('watertemp').textContent = data.watertemp;
			var waterTemperature = parseInt(data.watertemp, 10); 
    		charts[3].data.datasets[0].data.shift();
    		charts[3].data.datasets[0].data.push(waterTemperature);
    		charts[3].update();
		}
//	}
}
