'use strict'

//feather.replace({ 'aria-hidden': 'true' })

var graphTest={
  "xmax": 1669730010216,
  "xmin": 1669729990216,
  "formatX": "HH:MM:SS",
  "formatY": "us",
  "legend": {
    "text": "Timers",
    "color": "#0000ff"
  },
  "datasets": [
    {
      "name": "decode",
      "color": "#dd8452",
      "datax": [1669729990212,1669729990229,1669729990245,1669729990262],
      "datay": [5443,5433,5108,5272]
    }
  ]
};

function CreateChart(ctx) {
	let chart=new Chart(ctx,{
		type: 'line',
		data: {
			datasets: [{
				label: 'na',
				backgroundColor: "#000000",
				borderColor: "#000000",
				data: []
			}],
		},
		options: {
			animation: false,
			scales: {
				x: {
					type: 'time',
                    time: {
						tooltipFormat: 'YY-MM-dd HH:mm',
						displayFormats: {
							millisecond: 'HH:mm:ss.SSS',
							second: 'HH:mm:ss',
							minute: 'HH:mm',
							hour: 'HH'
						}
					},
 					ticks: {
						maxRotation: 70,
						minRotation: 70
					}
				},
				y: {
					display: true,
					ticks: {
						callback: function(label, index, labels) {
							return label;
						}
					},
					title: {
						display: true,
						text: 'value'
					}
				}
			}
		}
	});
	return chart;
}

var g_charts=[];

function SetStatus(message) {
	//var str=JSON.stringify(message, null, 2); // spacing level = 2
	//console.log(str);
	let divContainer=document.getElementById("serverstatus");
	divContainer.innerHTML="";
	let cnt=0;
	for(let i=0;i!=message.status.length;i++) {
		var messageStatus=message.status[i];
		for (var property in messageStatus) {
			if(property!="graphs" && property!="schema")
				ApendPropsTable(divContainer,messageStatus[property],messageStatus.schema,false);
		}
		for(let j=0;j!=messageStatus.graphs.length;j++) {
			if(typeof messageStatus.graphs[j].legend.timers!='undefined')
				continue;
			let chartId="chart"+cnt++;
			var ctxElement=document.getElementById(chartId);
			if(ctxElement==null) {
				ctxElement=document.createElement('canvas');
				ctxElement.id=chartId;
				ctxElement.className="my-4 w-100";
				ctxElement.width="900";
				ctxElement.height="280";
				divContainer.appendChild(ctxElement);
				var ctx=ctxElement.getContext('2d');
				ctxElement.chart=CreateChart(ctx);
			}
			//SetChart(ctxElement.chart,graphTest);
			SetChart(ctxElement.chart,messageStatus.graphs[j]);
		}
	}
}

function SetChart(chart,graph) {
	var hiddenDatasets={};
	for(var i=0; i<chart.data.datasets.length; i++) {
		if(!chart.isDatasetVisible(i)) {
			hiddenDatasets[chart.data.datasets[i].label]=1;
		}
	}
	chart.data.datasets=[];
	chart.options.scales.y.title.text=graph.legend.text;
	chart.data.datasets.length=graph.datasets.length;
	for(let i=0;i!=graph.datasets.length;i++) {
		chart.data.datasets[i]={label:"serie1",backgroundColor:"#FF2D00",borderColor:"#FF3D00",data:[]};
		chart.data.datasets[i].label=graph.datasets[i].name;
		chart.data.datasets[i].borderColor=graph.datasets[i].color;
		chart.data.datasets[i].backgroundColor=graph.datasets[i].color;
		chart.data.datasets[i].hidden=hiddenDatasets[graph.datasets[i].name];
		chart.data.datasets[i].data=[];
		if(graph.formatY=="us") {
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return label.toFixed(0)+' us';
			}};
		}else
		if(graph.formatY=="bytes") {
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return label.toFixed(0)+' bytes';
			}};
		}else
		if(graph.formatY=="kb") {
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return label.toFixed(1)+' kb';
			}};
		}else
		if(graph.formatY=="mb") {
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return label.toFixed(1)+' mb';
			}};
		}else
		if(graph.formatY=="kbs") {
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return label.toFixed(1)+' kB/s';
			}};
		}else
		if(graph.formatY=="mbits") {
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return label.toFixed(2)+' Mbit/s';
			}};

		}else
		if(graph.formatY=="int") {
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return label.toFixed(0);
			}};
		}else{
			chart.options.scales.y.ticks={
				callback: function(label, index, labels) {
				return GetLogTimeString(epochMillisecondsToJsDate(label));;
			}};
		}
		if(graph.datasets[i].datax.length!=graph.datasets[i].datay.length) {
			console.log("graph data error xlen:"+graph.datasets[i].datax.length+" ylen:"+graph.datasets[i].datay.length);
		}else{
			for(let j=0;j!=graph.datasets[i].datax.length;j++) {
				chart.data.datasets[i].data.push({x:graph.datasets[i].datax[j],y:graph.datasets[i].datay[j]});
			}
		}
	}
	chart.update();
}

$("#toggle").click(function() {
	 chartInstance.data.datasets.forEach(function(ds) {
    ds.hidden = !ds.hidden;
  });
  chartInstance.update();
});

function addData(chart,label,data) {
	chart.data.labels.push(label);
	chart.data.datasets.forEach((dataset) => {
		dataset.data.push(data);
	});
	chart.data.datasets.forEach((dataset) => {
		dataset.data.shift();
	});
	chart.data.labels.shift();
	chart.update();
}

function bin2String(array,offset,len) {
	var result='';
	for (var i=0; i < len; i++) {
		result += String.fromCharCode(array[i + offset]);
	}
	return result;
}

function addEventHandler(elem,eventType,handler) {
	if(elem.addEventListener) elem.addEventListener(eventType,handler,false);
	else if(elem.attachEvent) elem.attachEvent('on'+eventType,handler); 
	else return 0;
	return 1;
}

function RemapParameters(params,obj,stack,depth) {
	for (var property in obj) {
		if (obj.hasOwnProperty(property)) {
			let path=stack;
			if (path.charAt(0) === '.') {
				path=path.substring(1);
			}
			if (path.length)
				path += '.';
			if (typeof obj[property] == "object") {
				params.push({ path: path,depth: depth,property: property,isArray: Array.isArray(obj[property]) });
				RemapParameters(params,obj[property],stack + '.' + property,depth + 1);
			} else {
				//if (typeof obj[property] === 'string' || obj[property] instanceof String) {
				params.push({ path: path,depth: depth,property: property,value: obj[property] });
			}
		}
	}
}

function inputNoEnter(e) {
	if(e.charCode!=13) {
		return true;
	}
	e.returnValue=false;
	return false;
}

function inputDigitsOnly(e) {
	var chrTyped,chrCode=0,evt=e?e:event;
	if(evt.charCode!=null)     chrCode=evt.charCode;
	else if(evt.which!=null)   chrCode=evt.which;
	else if(evt.keyCode!=null) chrCode=evt.keyCode;
	if(chrCode==0) chrTyped='SPECIAL KEY';
	else chrTyped=String.fromCharCode(chrCode);
	//[test only:] display chrTyped on the status bar 
	self.status='inputDigitsOnly: chrTyped='+chrTyped;
	//Digits,special keys & backspace [\b] work as usual:
	if(chrTyped.match(/\d|[\b]|SPECIAL/)) return true;
	if(evt.altKey || evt.ctrlKey ||(chrCode!=13 && chrCode<28)) return true;			//13=enter
	//if(evt.altKey || evt.ctrlKey ||(chrCode<28)) return true;			//13=enter

	//if(e.charCode==13) {
	//	if(evt.preventDefault) evt.preventDefault();
		//evt.returnValue=false;
		//return true;
	//	return false;
	//}

	if(chrTyped=='.')return true;
	//Any other input? Prevent the default response:
	if(evt.preventDefault) evt.preventDefault();
	evt.returnValue=false;
	return false;
}

function ResetPropsTable(elementId) {
	let divContainer=document.getElementById(elementId);
	divContainer.innerHTML="";
}
function ApendPropsTable(divContainer,jsonObject,schema,defaultEdit) {
	let params=[];
	RemapParameters(params,jsonObject,'',0);
	let table=document.createElement("table");
	//table.className="table";
	table.className="table table-striped table-sm";
	let layoutRemap=[];
	for (let i=0; i != schema.length; i++) {
		layoutRemap[schema[i].param]=i;
	}
	let col=[];
	col.push('Parameter');
	col.push('Value');
	let tr=table.insertRow(-1);
	let widths=['50%','50%'];
	for (let i=0; i < col.length; i++) {
		let th=document.createElement("th");
		th.innerHTML=col[i];
		th.style.width=widths[i];
		tr.appendChild(th);
	}
	for (let i=0; i < params.length; i++) {
		if (typeof params[i].value == 'undefined')		//Is object?
			continue;
		let layout={type:'float',edit: defaultEdit};
		if(typeof params[i].value === 'string' || params[i].value instanceof String) {
			layout.type='string';
		}else
			if(typeof params[i].value === 'boolean') {
				layout.type='bool';
			}
		if (typeof layoutRemap[params[i].path + params[i].property] !== 'undefined') {
			let layoutIndex=layoutRemap[params[i].path + params[i].property];
			layout=schema[layoutIndex];
			if (typeof layout.edit == 'undefined')
				layout.edit=defaultEdit;
		}
		tr=table.insertRow(-1);
		let tabCell=tr.insertCell(-1);
		tabCell.innerHTML=(params[i].path + params[i].property).replace(/ /g,"&nbsp;");
		tabCell=tr.insertCell(-1);
		tabCell.m_layout=layout;
		tabCell.m_paramIndex=i;
		switch (layout.type) {
			case "float": {
				tabCell.innerHTML=parseFloat(params[i].value);
				break;
			}
			case "int": {
				tabCell.innerHTML=params[i].value.toFixed(0);
				break;
			}
			case "bool": {
				tabCell.innerHTML=params[i].value ? true : false;
				break;
			}
			case "string": {
				tabCell.innerHTML=params[i].value.replace(" ","&nbsp;");
				break;
			}
			case "float_range": {
				tabCell.innerHTML=params[i].value;
				break;
			}
			case "date": {
				let d=new Date(0); // The 0 there is the key,which sets the date to the epoch
				d.setUTCSeconds(params[i].value/1000);
				tabCell.innerHTML=GetDateTimeString(d);
				break;
			}
			case "dateus": {
				let d=new Date(0); // The 0 there is the key,which sets the date to the epoch
				d.setUTCMilliseconds(params[i].value/1000);
				tabCell.innerHTML=GetLogTimeString(d);
				break;
			}
			default: {
				tabCell.innerHTML=params[i].value;
			}
		}
		if (layout.edit) {
			tabCell.contentEditable="true";
			addEventHandler(tabCell,"keypress",function (e) {
				switch (this.m_layout.type) {
					case 'float': {
						return inputDigitsOnly(e);
					}
					case 'int': {
						return inputDigitsOnly(e);
					}
					case 'bool': {
						return inputNoEnter(e);
					}
					case 'string': {
						return inputNoEnter(e);
					}
					case 'float_range': {
						return inputDigitsOnly(e);
					}
					case 'int_range': {
						return inputDigitsOnly(e);
					}
				}
			});
			addEventHandler(tabCell,"focusout",function (e) {
				var value=this.textContent;
				switch (this.m_layout.type) {
					case 'float': {
						if (value.length == 0) value=0;
						value=parseFloat(value);
						break;
					}
					case 'int': {
						if (value.length == 0) value=0;
						value=parseInt(value,10);
						break;
					}
					case 'bool': {
						value=(value == true || value == 'true' || value == 1 || value == '1');
						break;
					}
					case 'float_range': {
						value=parseFloat(value);
						if (value < this.m_layout.range[0]) {
							value=this.m_layout.range[0];
						} else
							if (value > this.m_layout.range[1]) {
								value=this.m_layout.range[1];
							}
						break;
					}
					case 'int_range': {
						value=parseInt(value,10);
						if (value < this.m_layout.range[0]) {
							value=this.m_layout.range[0];
						} else
							if (value > this.m_layout.range[1]) {
								value=this.m_layout.range[1];
							}
						break;
					}
				}
				this.textContent=value;
			});
		}
	}
	divContainer.appendChild(table);
	return params;
}

var g_intervalHandle=0;
/*
function requestStatus() {
	if(connectionReady)
		g_connection.send(JSON.stringify({ type:"local",requestdata:true}));
}
function BeginStatus() {
	console.log("BeginStatus");
	if(connectionReady)
		g_connection.send(JSON.stringify({ type:"local",requestdata:true}));
	g_intervalHandle=setInterval(requestStatus,500);
}
function EndStatus() {
	clearInterval(g_intervalHandle);
	console.log("EndStatus");
}
*/
function BeginDashboard() {
	SetBreadCrumbs(["Home","Dashboard"]);
	console.log("BeginDashboard");
	if(connectionReady)
		g_connection.send(JSON.stringify({ type:"dashboard",dashboard:true}));
	g_intervalHandle=setInterval(function(){
		g_connection.send(JSON.stringify({ type:"dashboard",dashboard:true}));
	},1000);
}
function EndDashboard() {
	console.log("EndDashboard");
	clearInterval(g_intervalHandle);
}
function MessageDashboard(message) {
	//var str=JSON.stringify(message, null, 2); // spacing level = 2
	//console.log("MessageDashboard:"+str);
	let divContainer=document.getElementById("tabledashboard");
	divContainer.innerHTML="";
	let schema=[{param:"time",type:"dateus"}];
	ApendPropsTable(divContainer,message,schema,false);
}
/*
function BeginProperties() {
	console.log("BeginProperties");
	if(connectionReady)
		g_connection.send(JSON.stringify({ type:"remote",remoteData:true }));
}
function EndProperties() {
	console.log("EndProperties");
}
function MessageProperties(message) {
	let divContainer=document.getElementById("tableremotes");
	divContainer.innerHTML="";
	let schema=[];
	ApendPropsTable(divContainer,message.connections,schema,false);
}
*/
