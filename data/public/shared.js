'use strict'

function StringFirstToUpperCase(i) {
	return i.charAt(0).toUpperCase() + i.slice(1);
}
function CopyFromParameters(params,obj,stack,depth,paramIndex) {
	for (var property in obj) {
		if (obj.hasOwnProperty(property)) {
			let path=stack;
			if (path.charAt(0) === '.') {
				path=path.substring(1);
			}
			if (path.length)
				path += '.';
			if (typeof obj[property] == "object") {
				//params.push({ path: path,depth: depth,property: property,isArray: Array.isArray(obj[property]) });
				paramIndex=CopyFromParameters(params,obj[property],stack + '.' + property,depth+1,paramIndex+1);
			} else {
				obj[property]=params[paramIndex].value;
				paramIndex++;
				//params.push({ path: path,depth: depth,property: property,value: obj[property] });
			}
		}
	}
	return paramIndex;
}
function GetCookie(cname) {
	var name = cname + "=";
	var decodedCookie = decodeURIComponent(document.cookie);
	var ca = decodedCookie.split(';');
	for(var i = 0; i <ca.length; i++) {
	var c = ca[i];
	while (c.charAt(0) == ' ') {
		c = c.substring(1);
	}
	if (c.indexOf(name) == 0) {
		return c.substring(name.length,c.length);
	}
	}
	return "";
}
function DeleteCookie(cname) {
		document.cookie = name + '=;expires=Thu,01 Jan 1970 00:00:01 GMT;';
}
function SetCookie(cname,cvalue,exseconds) {
	var d=new Date();
	d.setTime(d.getTime()+(exseconds*1000));
	var expires="expires="+d.toUTCString();
	document.cookie=cname+"="+cvalue+";"+expires+";path=/";
}
function GetSession() {
	let c=GetCookie("s");
	if(typeof c=="undefined")
		return 0;
	if(c=="")
		return 0;
	return JSON.parse(c);
}

function GetDateTimeString(now) {
	var year=now.getFullYear();
	var month=now.getMonth()+1;
	var day=now.getDate();
	var hour=now.getHours();
	var minute=now.getMinutes();
	var second=now.getSeconds();
	if(month.toString().length==1) {
		month='0'+month;
	}
	if(day.toString().length==1) {
		day='0'+day;
	}
	if(hour.toString().length==1) {
		hour='0'+hour;
	}
	if(minute.toString().length==1) {
		minute='0'+minute;
	}
	if(second.toString().length==1) {
		second='0'+second;
	}
	var dateTime=day+'&#8209;'+month+'&#8209;'+year+'&nbsp;'+hour+':'+minute+':'+second;
	return dateTime;
}

function GetLogTimeString(now) {
	var year=now.getFullYear();
	var month=now.getMonth()+1;
	var day=now.getDate();
	var hour=now.getHours();
	var minute=now.getMinutes();
	var second=now.getSeconds();
	var millisecond=now.getMilliseconds();
	if(month.toString().length==1) {
		month='0'+month;
	}
	if(day.toString().length==1) {
		day='0'+day;
	}
	if(hour.toString().length==1) {
		hour='0'+hour;
	}
	if(minute.toString().length==1) {
		minute='0'+minute;
	}
	if(second.toString().length==1) {
		second='0'+second;
	}
	//var dateTime=year+'&nbsp;'+month+'&#8209;'+day+'&#8209;'+hour+':'+minute+':'+second+'.'+millisecond;
	var dateTime=year+'/'+month+'/'+day+' '+hour+':'+minute+':'+second+'.'+millisecond;
	return dateTime;
}

function epochSecondsToJsDate(ts){
    // ts=epoch timestamp
    // returns date obj
    return new Date(ts*1000);
}

function jsDateToEpochSeconds(d){
    // d=javascript date obj
    // returns epoch timestamp
    return (d.getTime()-d.getMilliseconds())/1000;
}

function epochMillisecondsToJsDate(ts){
    return new Date(ts);
}
function jsDateToEpochMilliseconds(d){
    return (d.getTime()-d.getMilliseconds());
}
