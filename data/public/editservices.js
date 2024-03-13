'use strict'

var g_editServices=null;
var g_handleUpdateTimeout=null;

function UpdateStatus() {
	if(g_editServices && typeof(g_editServices)!='undefined' && typeof(g_editServices.m_selectedStatusName)!='undefined' && g_editServices.m_selectedStatusName.length) {
		//console.log("UpdateStatus");
		SendGetStatus();
	}else{
		console.log("Unable to update status!");
	}
}

function BeginServiceStatus() {
	SetBreadCrumbs(["Home","Services","Status"]);
	document.getElementById("statustable").innerHTML="";
	document.getElementById("servicesdiv").style.display="none";
	document.getElementById("statusdiv").style.display="block";
	document.getElementById("propertiesdiv").style.display="none";
	let backstatusButtonEle=document.getElementById("backstatus");
	backstatusButtonEle.onclick=function() {
		EndServiceStatus();
		SetBreadCrumbs(["Home","Services"]);
		g_editServices={m_services:[],m_params:[]};
		UpdateServicesTable();
	}
	UpdateStatus();
}
function EndServiceStatus() {
	g_editServices=0;
	if(g_handleUpdateTimeout) {
		clearTimeout(g_handleUpdateTimeout);
		g_handleUpdateTimeout=null;
	}
	document.getElementById("statustable").innerHTML="";
	document.getElementById("servicesdiv").style.display="block";
	document.getElementById("statusdiv").style.display="none";
	document.getElementById("propertiesdiv").style.display="none";
}

function BeginPropertiesStatus() {
	SetBreadCrumbs(["Home","Services","Properties"]);
	document.getElementById("propertiestable").innerHTML="";
	document.getElementById("servicesdiv").style.display="none";
	document.getElementById("statusdiv").style.display="none";
	document.getElementById("propertiesdiv").style.display="block";
	let backpropertiesButtonEle=document.getElementById("backproperties");
	g_editServices.m_table=null;

	backpropertiesButtonEle.onclick=function() {
		EndPropertiesStatus();
		SetBreadCrumbs(["Home","Services"]);
		g_editServices={m_services:[],m_params:[]};
		UpdateServicesTable();
	}
	let updatePropertiesButtonEle=document.getElementById("updateproperties");
	updatePropertiesButtonEle.onclick=function() {
		//console.log("Before values "+JSON.stringify(g_editServices.m_settings));
		WriteKeyValues(g_editServices.m_params,g_editServices.m_settings,'',0,0);
		console.log("Values:"+JSON.stringify(g_editServices.m_settings));
	}
	SendGetProperties();
}
function EndPropertiesStatus() {
	document.getElementById("propertiestable").innerHTML="";
	document.getElementById("servicesdiv").style.display="block";
	document.getElementById("statusdiv").style.display="none";
	document.getElementById("propertiesdiv").style.display="none";
}

function SendGetStatus() {
	SendRequest("GetStatus",{id:g_editServices.m_selectedStatusName,session:GetSession()},function(response) {
		if(response.success==false) {
			UpdateWebLog("query GetStatus "+g_editServices.m_selectedStatusName+" failed");
			return;
		}
		document.getElementById("statustitle").innerHTML="Status: "+g_editServices.m_selectedStatusName;
		let divContainer=document.getElementById("statustable");
		UpdateStatusTable("statustable",response.settings,response.schema,false);
		if(typeof response.graphs!='undefined') {
			//console.log("graphs");
			let cnt=0;
			for(let j=0;j!=response.graphs.length;j++) {
				if(typeof response.graphs[j].legend.timers!='undefined')
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
				SetChart(ctxElement.chart,response.graphs[j]);
			}
		}
		g_handleUpdateTimeout=setTimeout(UpdateStatus,1000);
	});
}
function UpdateStatusTable(elementId,settings,schema,allowEdit) {
	g_editServices.m_settings={};
	Object.assign(g_editServices.m_settings,settings);
	g_editServices.m_schema=schema;
	g_editServices.m_params=CreatePropsTableService(elementId,g_editServices.m_settings,g_editServices.m_schema,allowEdit);
}

//Services
function ShowEditServices() {
	ClearPropsTableService("servicespropstable");
	ClearServicesJSONTable("editservicestable");
	document.getElementById("servicestitle").innerHTML="Services:";
	document.getElementById("updateservices").style.display="none";
	document.getElementById("editserviceslistdiv").style.display="block";
}

function BeginEditServices() {
	SetBreadCrumbs(["Home","Services"]);
	document.getElementById("servicesdiv").style.display="block";
	document.getElementById("statusdiv").style.display="none";
	document.getElementById("propertiesdiv").style.display="none";
	g_editServices={m_services:[],m_params:[]};
	UpdateServicesTable();
}
function EndEditServices() {
	g_editServices=null;
	document.getElementById("servicesdiv").style.display="none";
	document.getElementById("statusdiv").style.display="none";
	document.getElementById("propertiesdiv").style.display="none";
}

function RemoveServicePopup(id) {
	if( id==GetCookie("n")) {
		alert("Cannot erase active service");
		return;
	}
	$("#removeservicebut").unbind( "click" );
	$("#removeservicebut").click(function() {
		SendRequest("DeleteService",{id:id,session:GetSession()},function (response) {
			if(response.success==false) {
				UpdateWebLog("DeleteService "+id+" failed");
				bodyText.innerHTML="Remove service failed";
				$("#removeservicebut").hide();
			}else{
				UpdateWebLog("DeleteService "+id+" success");
				UpdateServicesTable();
				$("#modal-removeservice").modal('hide');
			}
		});
	});
	var bodyText=document.getElementById("modal-removeservice-bodytext");
	bodyText.innerHTML="Remove service "+id;
	var myModal=new bootstrap.Modal(document.getElementById("modal-removeservice"),{});
	myModal.show();
}
function SendGetProperties() {
	SendRequest("GetProperties",{id:g_editServices.m_selectedPropertiesName,session:GetSession()},function(response) {
		if(response.success==false) {
			UpdateWebLog("query GetProperties "+g_editServices.m_selectedPropertiesName+" failed");
			return;
		}
		document.getElementById("propertiestitle").innerHTML="Properties: "+g_editServices.m_selectedPropertiesName;
		UpdatePropertiesTable(response.settings,response.schema);
	});
}
function UpdatePropertiesTable(values,schema) {
	g_editServices.m_settings={};
	Object.assign(g_editServices.m_settings,values);
	//console.log("schema "+JSON.stringify(schema));
	g_editServices.m_table=CreateTableFromJsonWithSchema(g_editServices.m_settings,schema,function(key,value) {
		//console.log("change key:"+key+" val:"+value);
		g_editServices.m_params[key].value=value;
	});
	let divContainer=document.getElementById("propertiestable");
	divContainer.innerHTML="";
	divContainer.appendChild(g_editServices.m_table);
}
function UpdateServicesTable() {
	SendRequest("GetServices",{session:GetSession()},function(response) {
		let id=GetCookie("n");
		//console.log("ActiveServices response "+JSON.stringify(response));
		let activeServiceSettings={};
		var types={id:'string',created:'date',deviceId:'string',class:'string',IPv4:'string',watch:'watch',edit:'edit',remove:'trashcan'};
		var array=[];
		g_editServices.m_services=[];
		for(let i=0;i!=response.services.length;i++) {
			let service=response.services[i];
			let settings=service.settings;
			array.push({id:service.id,deviceId:service.deviceId,class:service.class,created:settings.created,IPv4:settings.host,watch:true,edit:true,remove:true});
			if(response.services[i].id==id) {
				activeServiceSettings=settings;
			}
			g_editServices.m_services.push({m_service:service});
		}
		CreateServicesJSONTable("editservicestable",types,array,activeServiceSettings.type,function(ele,col,row,type){
			//console.log("element "+col+","+row+","+type);
			switch(type) {
				case 'remove':{
					RemoveServicePopup(g_editServices.m_services[row].m_service.id);
					break;
				}
				case 'edit':{
					g_editServices.m_selectedPropertiesName=g_editServices.m_services[row].m_service.id;
					BeginPropertiesStatus();
					break;
				}
				case 'watch':{
					g_editServices.m_selectedStatusName=g_editServices.m_services[row].m_service.id;
					BeginServiceStatus();
					break;
				}
			}
		});
	});
}


function CreateNewService(id) {
	//console.log("CreateNewService" +id);
	let s=JSON.parse(GetCookie("s"));
	SendRequest("SetService",{id:id,session:s},function(response) {
		if(response.success!=true) {
			UpdateWebLog("Create Service "+id+" failed");
		}else{
			UpdateWebLog("Create Service "+id+" success");
		}
		UpdateServicesTable();
	});
}

function ClearPropsTableService(elementId) {
	let divContainer=document.getElementById(elementId);
	divContainer.innerHTML="";
}


function CreatePropsTableService(elementId,jsonObject,schema,allowEdit) {
	let params=[];
	RemapParameters(params,jsonObject,'',0);
	let table=document.createElement("table");
	table.className="table table-striped table-sm";
	let layoutRemap=[];
	for(let i=0; i !=schema.length; i++) {
		layoutRemap[schema[i].param]=i;
	}
	let col=[];
	col.push('Parameter');
	col.push('Value');

	let tableHead=table.createTHead();

	let tr=tableHead.insertRow(-1);
	let widths=['50%','50%'];
	for(let i=0;i<col.length;i++) {
		let th=document.createElement("th");
		th.innerHTML=col[i];
		th.style.width=widths[i];
		tr.appendChild(th);
	}

	let tableBody=table.createTBody();

	for(let i=0;i<params.length;i++) {
		if(typeof params[i].value=='undefined')		//Is object?
			continue;
		let layout={type:'float',edit:true};
		if (typeof params[i].value === 'string' || params[i].value instanceof String) {
			layout.type='string';
		}else
		if (typeof params[i].value === 'boolean') {
			layout.type='bool';
		}
		if(typeof layoutRemap[params[i].path+params[i].property] !=='undefined') {
			let layoutIndex=layoutRemap[params[i].path + params[i].property];
			layout=schema[layoutIndex];
			if(typeof layout.edit=='undefined')
				layout.edit=true;
		}
		tr=tableBody.insertRow(-1);
		let tabCell=tr.insertCell(-1);
		tabCell.innerHTML=(params[i].path + params[i].property).replace(/ /g,"&nbsp;");
		tabCell=tr.insertCell(-1);
		tabCell.m_layout=layout;
		tabCell.m_paramIndex=i;
		switch(layout.type) {
			case "float": {
				tabCell.innerHTML=parseFloat(params[i].value);
				break;
			}
			case "int": {
				tabCell.innerHTML=params[i].value.toFixed(0);
				break;
			}
			case "bool": {
				tabCell.innerHTML=params[i].value ? true:false;
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
			case "int_range": {
				tabCell.innerHTML=params[i].value;
				break;
			}
			case "date": {
				let d=new Date(0); // The 0 there is the key,which sets the date to the epoch
				d.setUTCSeconds(params[i].value/1000);
				tabCell.innerHTML=GetDateTimeString(d);
				break;
			}
			case "bcrypt": {
				tabCell.innerHTML=params[i].value.replace(" ","&nbsp;");
				break;
			};
			default: {
				tabCell.innerHTML=params[i].value;
			}
		}
	}
	let divContainer=document.getElementById(elementId);
	divContainer.innerHTML="";
	divContainer.appendChild(table);
	return params;
}

function ClearServicesJSONTable(elementId) {
	let divContainer=document.getElementById(elementId);
	divContainer.innerHTML="";
}
function CreateServicesJSONTable(elementId,typesList,elements,type,clickFunction) {
	let table=document.createElement("table");
	//table.className="table table-hover table-striped table-sm";
	table.className="table table-sm table-striped table-hover";
	let col=[];
	for(let i=0;i<elements.length;i++) {
		for(let key in elements[i]) {
			if(!(key in typesList)) {
				continue;
			}
			if(col.indexOf(key)===-1) {
				col.push(key);
			}
		}
	}
	let tableHead=table.createTHead();
	let tr=tableHead.insertRow(-1);
	for(let i=0;i<col.length;i++) {
		let th=document.createElement("th");
		th.innerHTML=StringFirstToUpperCase(col[i]);
		tr.appendChild(th);
	}
	let tableBody=table.createTBody();
	for(let i=0;i<elements.length;i++) {
		tr=tableBody.insertRow(-1);
		for(let j=0;j<col.length;j++) {
			let tabCell=tr.insertCell(-1);
			tabCell.row=i;
			tabCell.col=j;
			tabCell.type=col[j];
			tabCell.onclick=function(){
				clickFunction(this,this.col,this.row,this.type);
			};
			switch(typesList[col[j]]) {
				case "string": {
					tabCell.innerHTML=elements[i][col[j]];//.replace(/ /g,"&nbsp;");
					break;
				}
				case "int": {
					tabCell.innerHTML=elements[i][col[j]].toFixed(0);
					break;
				}
				case "float": {
					tabCell.innerHTML=elements[i][col[j]];
					break;
				}
				case "date": {
					var d=new Date(0); // The 0 there is the key,which sets the date to the epoch
					d.setUTCSeconds(elements[i][col[j]]/1000);
					tabCell.innerHTML=GetDateTimeString(d);
					break;
				}
				case "check-disabled": {
					if(elements[i][col[j]]) {
						tabCell.innerHTML="<i class='fa fa-check disabled'></i>";
					}else{
						tabCell.innerHTML="<i class='fa  disabled'></i>";
					}
					break;
				}
				case "check": {
					if(elements[i][col[j]]) {
						tabCell.innerHTML="<i class='fa fa-check-square-o'></i>";
					}else{
						tabCell.innerHTML="<i class='fa  fa-square-o'></i>";
					}
					break;
				}
				case "trashcan": {
					if(elements[i][col[j]]) {
						tabCell.innerHTML="<i class='fa fa-trash'></i>";
					}else{
						tabCell.innerHTML="<i class='fa fa-trash disabled'></i>";
					}
					break;
				}
				case "edit": {
					if(elements[i][col[j]]) {
						tabCell.innerHTML="<i class='fa fa-edit'></i>";
					}else{
						tabCell.innerHTML="<i class='fa fa-edit disabled'></i>";
					}
					break;
				}
				case "watch": {
					if(elements[i][col[j]]) {
						tabCell.innerHTML="<i class='fa fa-bar-chart'></i>";
					}else{
						tabCell.innerHTML="<i class='fa fa-bar-chart disabled'></i>";
					}
					break;
				}
				default: {
					tabCell.innerHTML="NA";
					break;
				}
			}
		}
	}
	let divContainer=document.getElementById(elementId);
	divContainer.innerHTML="";
	divContainer.appendChild(table);
}

function SchemaFormatIntegerToHtml(cell,key,value,schema) {
	value=parseInt(value);
	let htmlString="";
	if(typeof schema.properties!='undefined' && typeof schema.properties[key]!='undefined') {
		let property=schema.properties[key];
		cell.m_property=property;
		if(typeof property.items!='undefined') {
			htmlString="<select class='form-select form-select-sm  w-75'>";
			for(let i=0;i!=property.items.enum.length;i++) {
				if(property.items.enum[i]==value) {
					htmlString+="<option value='"+i+"' selected>"+parseInt(property.items.enum[i])+"</option>";
				}else{
					htmlString+="<option value='"+i+"'>"+parseInt(property.items.enum[i])+"</option>";
				}
			}
			htmlString+="</select>";
			addEventHandler(cell,"focusout", function(e) {
				this.m_changeValue(this.m_key,this.m_property.items.enum[e.target.value]);
			});
		}else{
			htmlString="<div class=''><input class='form-control form-control-sm w-75' value='"+value+"' type='text'></input></div>";
			cell.contentEditable="true";
			addEventHandler(cell,"keypress",function(e) {
				return inputDigitsOnly(e);
			});
			addEventHandler(cell,"focusout", function(e) {
				let value=parseInt(e.target.value);
				if(typeof this.m_property.maximum!='undefined')
					value=value>this.m_property.maximum ? this.m_property.maximum:value;
				if(typeof this.m_property.minimum!='undefined')
					value=e.target.value<this.m_property.minimum ? this.m_property.minimum:value;
				e.target.value=value;
				this.m_changeValue(this.m_key,value);
			});
		}
	}else{
		//htmlString="<div class=''><input class='form-control form-control-sm w-75' value='"+value+"' type='text'></input></div>";
		htmlString=value;
	}
	cell.innerHTML=htmlString;
}

function SchemaFormatNumberToHtml(cell,key,value,schema) {
	let htmlString="";
	value=parseFloat(value);
	if(typeof schema.properties!='undefined' && typeof schema.properties[key]!='undefined') {
		let property=schema.properties[key];
		cell.m_property=property;
		if(typeof property.items!='undefined') {
			htmlString="<select class='form-select form-select-sm  w-75'>";
			for(let i=0;i!=property.items.enum.length;i++) {
				if(property.items.enum[i]==value) {
					htmlString+="<option value='"+i+"' selected>"+parseFloat(property.items.enum[i])+"</option>";
				}else{
					htmlString+="<option value='"+i+"'>"+parseFloat(property.items.enum[i])+"</option>";
				}
			}
			htmlString+="</select>";
			addEventHandler(cell,"focusout", function(e) {
				this.m_changeValue(this.m_key,this.m_property.items.enum[e.target.value]);
			});
		}else{
			htmlString="<div class=''><input class='form-control form-control-sm w-75' value='"+value+"' type='text'></input></div>";
			addEventHandler(cell,"keypress",function(e) {
				return inputDigitsOnly(e);
			});
			addEventHandler(cell,"focusout", function(e) {
				let value=parseFloat(e.target.value);
				if(typeof this.m_property.maximum!='undefined')
					value=value>this.m_property.maximum ? this.m_property.maximum:value;
				if(typeof this.m_property.minimum!='undefined')
					value=e.target.value<this.m_property.minimum ? this.m_property.minimum:value;
				e.target.value=value;
				this.m_changeValue(this.m_key,value);
			});
		}
	}else{
		//htmlString="<div class=''><input class='form-control form-control-sm w-75' value='"+value+"' type='text'></input></div>";
		htmlString=value;
	}
	cell.innerHTML=htmlString;
}

function SchemaFormatStringToHtml(cell,key,value,schema) {
	let htmlString="";
	if(typeof schema.properties!='undefined' && typeof schema.properties[key]!='undefined') {
		let property=schema.properties[key];
		cell.m_property=property;
		if(typeof property.items!='undefined') {
			htmlString="<select class='form-select form-select-sm  w-75'>";
			for(let i=0;i!=property.items.enum.length;i++) {
				if(property.items.enum[i]==value) {
					htmlString+="<option value='"+i+"' selected>"+property.items.enum[i]+"</option>";
				}else{
					htmlString+="<option value='"+i+"'>"+property.items.enum[i]+"</option>";
				}
			}
			htmlString+="</select>";
		}else{
			htmlString="<div class=''><input class='form-control form-control-sm w-75' value='"+value+"' type='text'></input></div>";
		}
	}else{
		htmlString="<div class=''><input class='form-control form-control-sm w-75' value='"+value+"' type='text'></input></div>";
	}
	addEventHandler(cell,"change", function(e) {
		if(typeof(this.m_property)!='undefined' && typeof(this.m_property.items)!='undefined') {
			this.m_changeValue(this.m_key,this.m_property.items.enum[e.target.value]);
		}else{
			this.m_changeValue(this.m_key,e.target.value);
		}
	});
	cell.innerHTML=htmlString;
}
function SchemaFormatBooleanToHtml(cell,key,value,schema) {
	value=(value==true || value=='true' || value==1 || value=='1');
	let newDiv=document.createElement("div");
	newDiv.classList.add("form-check");
	newDiv.classList.add("form-check-sm");
	let newInput=document.createElement("input");
	newInput.classList.add("form-check-input");
	newInput.setAttribute("type","checkbox");
	newInput.checked=true;
	newDiv.appendChild(newInput);
	cell.appendChild(newDiv);
	newInput.m_cell=cell;
	cell.m_key=key;
	addEventHandler(newInput,"change", function(e) {
		//console.log("BooleanChange:"+e.target.checked);
		let cell=e.target.m_cell;
		cell.m_changeValue(cell.m_key,e.target.checked);
	});
}

function WriteKeyValues(params,obj,stack) {
	for(var key in obj) {
		if(params.hasOwnProperty(key)) {
			let path=stack;
			if(path.charAt(0) === '.') {
				path=path.substring(1);
			}
			if(path.length)
				path += '.';
			if(typeof obj[key] == "object") {
				WriteKeyValues(params,obj[key],stack + '.' + key);
			}else{
				obj[key]=params[key].value;
			}
		}else{
			console.log("WriteKeyValues key:"+key+" not found");
		}
	}
}

function ReadKeyValues(params,obj,stack,depth) {
	for(var key in obj) {
		if(obj.hasOwnProperty(key)) {
			let path=stack;
			if(path.charAt(0)==='.') {
				path=path.substring(1);
			}
			if(path.length)
				path += '.';
			if(typeof(obj[key])=="object") {
				params[key]={ path: path,depth: depth,key: key,isArray: Array.isArray(obj[key]) };
				ReadKeyValues(params,obj[key],stack + '.' + key,depth + 1);
			}else{
				params[key]={ path: path,depth: depth,key: key,value: obj[key] };
			}
		}else{
			console.log("ReadKeyValues key:"+key+" not found");
		}
	}
}

function CreateTableFromJsonWithSchema(values,schema,changeFunction) {
	let params={};
	ReadKeyValues(params,values,'',0);
	let table=document.createElement("table");
	table.className="table";
	let tableBody=table.createTBody();
	for(const [key, param] of Object.entries(params)) {
		//console.log("param:"+JSON.stringify(param));
		if(typeof(param.value)=='undefined')		//Is object?
			continue;
		let tr=tableBody.insertRow(-1);
		let cell=tr.insertCell(-1);
		let valueName=(param.path + param.key).replace(/ /g,"&nbsp;");
		cell.innerHTML="<div class='text-end align-middle fw-bold'>"+valueName+"</div>";
		cell.style.width="30%";
		cell=tr.insertCell(-1);
		cell.style.width="70%";
		cell.m_changeValue=changeFunction;
		cell.m_key=key;
		if(typeof schema.properties!='undefined' && typeof schema.properties[param.key]!='undefined') {
			let property=schema.properties[param.key];
			if(typeof property.type=='undefined') {
				cell.innerHTML="TYPE ERROR";
			}else{
				switch(property.type) {
					case "number": {
						SchemaFormatNumberToHtml(cell,param.key,param.value,schema);
						break;
					}
					case "integer": {
						SchemaFormatIntegerToHtml(cell,param.key,param.value,schema);
						break;
					}
					case "boolean": {
						SchemaFormatBooleanToHtml(cell,param.key,param.value,schema);
						break;
					}
					case "string": {
						SchemaFormatStringToHtml(cell,param.key,param.value,schema);
						break;
					}
				}
			}
			//console.log("property found "+JSON.stringify(property));
		}else{
			if(typeof param.value === 'string' || param.value instanceof String) {
				SchemaFormatStringToHtml(cell,param.ley,param.value,schema);
			}else
			if(typeof param.value === 'boolean') {
				SchemaFormatBooleanToHtml(cell,param.key,param.value,schema);
			}else{
				SchemaFormatNumberToHtml(cell,param.key,param.value,schema);
			}
		}
	}
	g_editServices.m_values=values;
	g_editServices.m_params=params;
	return table;
}
