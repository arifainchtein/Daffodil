var currentData;

$(document).ready(function() {
    refreshData();
    refreshScreen();
});
//
// end of init functions
//

//
// wifi functions
//
$(document).on('click','#wifi-switch', function (){
    var ckb = $("#wifi-switch").is(':checked');
    if(ckb){
        setStationMode();
    }else{
        setAccessPointMode();
    }
});
function setAccessPointMode(){
    $('#wifi-switch').prop('checked', false);
    $('.ap-mode').removeClass('d-none').addClass('d-block');
    $('.station-mode').removeClass('d-block').addClass('d-none');
    $('#ap-address').val(currentData.apaddress);
    $('#ap-password').val('');
    $('#ap-hostname').val(currentData.hostname);
}

function setStationMode(){
    var ssids = JSON.parse(currentData.ssids);
    $('.station-mode').removeClass('d-none').addClass('d-block');
    $('.ap-mode').removeClass('d-block').addClass('d-none');
    $('#ssid').find('option').remove().end();
    $('#ipaddress').html(currentData.ipAddress);
    $('#wifi-switch').prop('checked', true);
    var optionText;
    var color;
    $.each(ssids, function(i,ss) {   
        if(ss.rssi>-60 && ss.rssi<=-50){
            color='green';
        }else if(ss.rssi>-80 && ss.rssi<-60){
            color='yellow';
        }else if( ss.rssi<-80){
            color='red';
        }
        if(ss.enc){
            optionText = ss.ssid + '   ' +ss.rssi + '  ' +'pwd';
        }else{
            optionText = ss.ssid + '   ' +ss.rssi + '  ' +'';
        }
        $('#ssid').append($("<option>",{
                        value: ss.ssid,
                            text:optionText
                        }));
    });
    if(currentData.ssid!=''){
        $('#ssid').val(currentData.ssid);
    }
    $('#wifi-password').val('');
    $('#hostname').val(currentData.hostname);
    
}


function setStationMode1(){
    $.ajax({
        type: "GET",
        async:false,
        url: "/DaffodilServlet",
        data:{formName:"GetAvailableSSIDS"},
        success: function (result) {
            var ssids = JSON.parse(result);
            $('.station-mode').removeClass('d-none').addClass('d-block');
            $('.ap-mode').removeClass('d-block').addClass('d-none');
            $('#ssid').find('option').remove().end();

            $('#wifi-switch').prop('checked', true);
            
            var color;
            $.each(ssids, function(i,ss) {   
                

                if(ss.rssi>-60 && ss.rssi<=-50){
                    color='green';
                }else if(ss.rssi>-80 && ss.rssi<-60){
                    color='yellow';
                }else if( ss.rssi<-80){
                    color='red';
                }

                $('#ssid').append($("<option></option>")
                            .attr("value", ss.name)
                            .text(ss.name + '   ' +ss.rssi + '  ' + ss.enc)); 
                });
            $('#ssid').val(currentData.ssid);
            $('#wifi-password').val('');
            $('#hostname').val(currentData.hostname);
            
        },
        error: function(data){
            console.log("solo:" + JSON.stringify(data));
            alert("There was an error processing your request:" + JSON.stringify(data));
            return false;
        }
    });
    
}




$(document).on('click','#wifi-config-icon', function (){
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#wifi-configure').removeClass('d-none').addClass('d-block');
    
    if(currentData.stationmode){
        setStationMode();
    }else{
        setAccessPointMode();
    }
   
});

$(document).on('click','#wifi-configure-cancel', function (){
    refreshScreen();
 });

 $(document).on('click','#wifi-configure-submit', function (){
    var ckb = $("#wifi-switch").is(':checked');
    
    if(ckb){
        var ssid = $('#ssid').val();
        var pass=$('#wifi-password').val();
        var host =  $('#hostname').val();
        if(ssid== undefined || ssid==''){
            alert("Please input SSID");
            return false;
        };
        if(host== undefined || host==''){
            alert("Please input Host");
            return false;
        };
        
        $.ajax({
            type: "POST",
            url: "/DaffodilServlet",
            data: {formName:'ConfigSTA',ssid:ssid,pass:pass,host:host},
            success: function (result) {
                currentData = JSON.parse(result);
                refreshScreen();
            },
            error: function(data){
                console.log("solo:" + JSON.stringify(data));
                alert("There was an error processing your request:" + JSON.stringify(data));
                return false;
            }
        });
        
    }else{
        var apaddress = $('#ap-address').val();
        var pass=$('#ap-password').val();
        var host =  $('#ap-hostname').val();
        if(apaddress== undefined || apaddress==''){
            alert("Please input AP Address");
            return false;
        };
        if(host== undefined || host==''){
            alert("Please input Host");
            return false;
        };
        
        $.ajax({
            type: "POST",
            url: "/DaffodilServlet",
            data: {formName:'ConfigAP',apaddress:apaddress,pass:pass,host:host},
            success: function (result) {
                currentData = JSON.parse(result);
                refreshScreen();
            },
            error: function(data){
                console.log("solo:" + JSON.stringify(data));
                alert("There was an error processing your request:" + JSON.stringify(data));
                return false;
            }
        });

    }
    
    alert("Please wait 5 seconds and refresh");

   
   
 
  });
//
// end of wifi functions
//



//
// Flow Sensor 1
//
$(document).on('click','#flow-sensor-1-config-icon', function (){
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#flow-sensor-1-configure').removeClass('d-none').addClass('d-block');
    $('#flow-sensor-1-input-name').val(currentData.flow1name);
    $('#flow-sensor-1-qfactor-input').val(currentData.qfactor1);
    
});



$(document).on('click','#flow-sensor-1-configure-cancel', function (){
   refreshScreen();
});

$(document).on('click','#flow-sensor-1-configure-submit', function (){
   var flow1Name = $('#flow-sensor-1-input-name').val();
   var qfactor1 =  $('#flow-sensor-1-qfactor-input').val();
   $.ajax({
        type: "POST",
        url: "/DaffodilServlet",
        data:{formName:"SetSensor1Param",flow1Name:flow1Name,qfactor1:qfactor1},
        success: function (result) {
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: function(data){
            console.log("solo:" + JSON.stringify(data));
            alert("There was an error processing your request:" + JSON.stringify(data));
            return false;
        }
    });

 });


 //
 // end of flow sensor 1
 //

 //
// Flow Sensor 2
//
$(document).on('click','#flow-sensor-2-config-icon', function (){
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#flow-sensor-2-configure').removeClass('d-none').addClass('d-block');
    $('#flow-sensor-2-input-name').val(currentData.flow2name);
    $('#flow-sensor-2-qfactor-input').val(currentData.qfactor2);
    
});



$(document).on('click','#flow-sensor-2-configure-cancel', function (){
   refreshScreen();
});

$(document).on('click','#flow-sensor-2-configure-submit', function (){
   var flow2Name = $('#flow-sensor-2-input-name').val();
   var qfactor2 =  $('#flow-sensor-2-qfactor-input').val();
   $.ajax({
        type: "POST",
        url: "/DaffodilServlet",
        data:{formName:"SetFlowSensor2Param",flow2Name:flow2Name,qfactor2:qfactor2},
        success: function (result) {
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: function(data){
            console.log("solo:" + JSON.stringify(data));
            alert("There was an error processing your request:" + JSON.stringify(data));
            return false;
        }
    });

 });


 //
 // end of flow sensor 2
 //


//
// Tank Sensor 1
//
$(document).on('click','#tank-sensor-1-config-icon', function (){
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#tank-sensor-1-configure').removeClass('d-none').addClass('d-block');
    $('#tank-sensor-1-name-input').val(currentData.tank1name);
    $('#tank-sensor-1-height-input').val(currentData.tank1heightmeters);
    $('#tank-sensor-1-volume-input').val(currentData.tank1maxvollit);
    $('#tank-sensor-1-config-heading').html(currentData.tank1name + " Configuration");
});



$(document).on('click','#tank-sensor-1-configure-cancel', function (){
   refreshScreen();
});

$(document).on('click','#tank-sensor-1-configure-submit', function (){
   var tank1Name = $('#tank-sensor-1-name-input').val();
   var tank1heightmeters =  $('#tank-sensor-1-height-input').val();
   var tank1maxvollit =  $('#tank-sensor-1-volume-input').val();
   if(tank1Name==undefined || tank1Name==''){
    alert('Please input tank name');
    return false;
   }
   if(tank1heightmeters==0){
    alert('Tank height has to be greater than zero');
    return false;
   }
   if(tank1maxvollit==0){
    alert('Tank volume has to be greater than zero');
    return false;
   }
   $.ajax({
        type: "POST",
        url: "/DaffodilServlet",
        data:{formName:"SetTank1Param",tank1Name:tank1Name,tank1heightmeters:tank1heightmeters,tank1maxvollit:tank1maxvollit},
        success: function (result) {
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: function(data){
            console.log("solo:" + JSON.stringify(data));
            alert("There was an error processing your request:" + JSON.stringify(data));
            return false;
        }
    });

 });


 //
 // end of tank sensor 1
 //


//
// Tank Sensor 2
//
$(document).on('click','#tank-sensor-2-config-icon', function (){
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#tank-sensor-2-configure').removeClass('d-none').addClass('d-block');
    $('#tank-sensor-2-name-input').val(currentData.tank2name);
    $('#tank-sensor-2-height-input').val(currentData.tank2heightmeters);
    $('#tank-sensor-2-volume-input').val(currentData.tank2maxvollit);
    $('#tank-sensor-2-config-heading').html(currentData.tank2name + " Configuration");
});




 
$(document).on('click','#tank-sensor-2-configure-cancel', function (){
   refreshScreen();
});

$(document).on('click','#tank-sensor-2-configure-submit', function (){
   var tank2name = $('#tank-sensor-2-name-input').val();
   var tank2heightmeters =  $('#tank-sensor-2-height-input').val();
   var tank2maxvollit =  $('#tank-sensor-2-volume-input').val();
   if(tank2name==undefined || tank2name==''){
    alert('Please input tank name');
    return false;
   }
   if(tank2heightmeters==0){
    alert('Tank height has to be greater than zero');
    return false;
   }
   if(tank2maxvollit==0){
    alert('Tank volume has to be greater than zero');
    return false;
   }
   $.ajax({
        type: "POST",
        url: "/DaffodilServlet",
        data:{formName:"SetTank2Param",tank2name:tank2name,tank2heightmeters:tank2heightmeters,tank2maxvollit:tank2maxvollit},
        success: function (result) {
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: function(data){
            console.log("solo:" + JSON.stringify(data));
            alert("There was an error processing your request:" + JSON.stringify(data));
            return false;
        }
    });

 });


 //
 // end of tank sensor 1
 //


 function showTank1(){
    $('#tank-sensor-1-name').html(currentData.tank1name);
    $('#tank-sensor-1-value').html(currentData.tank1waterLevel || 0);
    $('#tank-sensor-1-units').html('liters');
    $('#tank-sensor-1').removeClass('d-none').addClass('d-block');
 }

function showTank2(){
    $('#tank-sensor-2-name').html(currentData.tank2name);
    $('#tank-sensor-2-value').html(currentData.tank2waterLevel || 0);
    $('#tank-sensor-2-units').html('liters');
    $('#tank-sensor-2').removeClass('d-none').addClass('d-block');
}
function showSensor1(){
    $('#flow-sensor-1-name').html(currentData.flow1name);
    $('#flow-sensor-1-total-heading').html(currentData.flow1name + ' Volume');
    $('#flow-sensor-1-value').html(currentData.flowrate);
    $('#flow-sensor-1-units').html('liters/min');
    $('#flow-sensor-1').removeClass('d-none').addClass('d-block');
    
    $('#flow-sensor-1-total').removeClass('d-none').addClass('d-block');

    var mils = currentData.totalmilliLitres;
    if(mils>10000){
        
        $('#flow-sensor-1-total-value').html((Math.round(mils ) / 1000).toFixed(0));
        $('#flow-sensor-1-total-units').html('liters');
    }else if(mils>1000){
        
        $('#flow-sensor-1-total-value').html((Math.round(mils ) / 1000).toFixed(2));
        $('#flow-sensor-1-total-units').html('liters');
    }else{
        $('#flow-sensor-1-total-value').html(mils);
        $('#flow-sensor-1-total-units').html('milliliters');
    }
}

function showSensor2(){
    $('#flow-sensor-2-name').html(currentData.flow2name);
    $('#flow-sensor-2-total-heading').html(currentData.flow2name + ' Volume');
    
        $('#flow-sensor-2-units').html('liters/min');
        $('#flow-sensor-2').removeClass('d-none').addClass('d-block');
        $('#flow-sensor-2-value').html(currentData.flowrate2);
        $('#flow-sensor-2-total').removeClass('d-none').addClass('d-block');
    
        var mils = currentData.totalmilliLitres2;
        if(mils>10000){
            
            $('#flow-sensor-2-total-value').html((Math.round(mils ) / 1000).toFixed(0));
            $('#flow-sensor-2-total-units').html('liters');
        }else if(mils>1000){
            
            $('#flow-sensor-2-total-value').html((Math.round(mils ) / 1000).toFixed(2));
            $('#flow-sensor-2-total-units').html('liters');
        }else{
            $('#flow-sensor-2-total-value').html(mils);
            $('#flow-sensor-2-total-units').html('milliliters');
        }
}

function showDaffodil(){
    $('#daffodil-sceptic-value').html(Math.round(currentData.scepticAvailablePercentage));
    $('#daffodil-sceptic-units').html('%');

    $('#daffodil-temp-value').html(Math.round(currentData.outdoortemperature));
    $('#daffodil-temp-units').html('C');
    $('#daffodil-capacitor-value').html(currentData.batteryVoltage.toFixed(2) + ' / ' + (currentData.v50Voltage || 0).toFixed(2));
    $('#daffodil-humidity-value').html(Math.round(currentData.outdoorhumidity));
    $('#daffodil-humidity-units').html('C%');
    $('#daffodil-sceptic').removeClass('d-none').addClass('d-block');
    $('#daffodil-temp').removeClass('d-none').addClass('d-block');
    $('#daffodil-humidity').removeClass('d-none').addClass('d-block');
    $('#daffodil-capacitor').removeClass('d-none').addClass('d-block');
}

$(document).on('click','#system-info-icon', function (){
    $('.display-module').removeClass('d-block').addClass('d-none');
    if($('#system-info').hasClass('d-block')){
        $('#system-info').removeClass('d-block').addClass('d-none');
    }else{
        $('#system-info').removeClass('d-none').addClass('d-block');
    }
    if(currentData.internetAvailable){
        $('#internet-time-configure-button').removeClass('d-none').addClass('d-block');
        $('#internet-ping-time').html(currentData.internetPingTime);
    }else{
        $('#internet-time-configure-button').removeClass('d-block').addClass('d-none');
        $('#internet-ping-time').html('');
    }
    if(currentData.latitude==null || currentData.latitude==undefined){
        $('#latitude').val('');
    }else{
        $('#latitude').val(currentData.latitude);
    }

    if(currentData.longitude==null || currentData.longitude==undefined){
        $('#longitude').val('');
    }else{
        $('#longitude').val(currentData.longitude);
    }
});

$(document).on('click','#system-info-cancel', function (){
    refreshScreen();
 });


$(document).on('click','#manual-time-button', function (){
    var date = new Date();
    var day = ("0" + date.getDate()).slice(-2);
    var month = ("0" + (date.getMonth() + 1)).slice(-2);
    var today = date.getFullYear() + "-" + (month) + "-" + (day) +'T'+  date.getHours() + ":"+ date.getMinutes()+":"+date.getSeconds();
    $('#date-input').val(today);
    $('#manual-time-modal').modal('show');
});



$(document).on('click','#manual-time-submit', function (){
    var date = new Date($('#date-input').val());
    if(date.getTime()<10000){
     alert('Please input valid time');
     return false;
    }
   
    var day = date.getDate();
    var month = (date.getMonth() + 1);
    var year = date.getYear()-100;
    var time = "SetTime#" + (day)+"#" +(month) +"#"+ year +"#"+(date.getDay()) + "#"+  date.getHours() + "#"+ date.getMinutes()+"#"+date.getSeconds();    
    $.ajax({
         type: "POST",
         url: "/DaffodilServlet",
         data:{formName:"ManualSetTime",time:time},
         success: function (result) {
            $('#manual-time-modal').modal('hide');
             currentData = JSON.parse(result);
             refreshScreen();
         },
         error: function(data){
             console.log("solo:" + JSON.stringify(data));
             alert("There was an error processing your request:" + JSON.stringify(data));
             return false;
         }
     });
  });

  $(document).on('click','#internet-time-configure-button', function (){
       $.ajax({
         type: "POST",
         url: "/DaffodilServlet",
         data:{formName:"SetTimeViaInternet"},
         success: function (result) {
             currentData = JSON.parse(result);
             refreshScreen();
         },
         error: function(data){
             console.log("solo:" + JSON.stringify(data));
             alert("There was an error processing your request:" + JSON.stringify(data));
             return false;
         }
     });
 
  });

  //
  // GPS: the Latitude/Longitude fields take decimal degrees (-33.859722) or degrees minutes
  // seconds as the iPhone Compass shows them (33°51′35″ S), or typed with spaces (33 51 35 S).
  // S and W (or a minus sign) make it negative. Converted to decimal before it is saved.
  //
  function parseCoordinate(text, positiveLetter, negativeLetter, max) {
      var s = String(text).trim().toUpperCase();
      if (s == '') return null;
      var sign = 1;
      var m = s.match(/^([NSEW])\s*(.*)$/) || s.match(/^(.*?)\s*([NSEW])$/);
      if (m) {
          var letter = /^[NSEW]$/.test(m[1]) ? m[1] : m[2];
          s = /^[NSEW]$/.test(m[1]) ? m[2] : m[1];
          if (letter == negativeLetter) sign = -1;
          else if (letter != positiveLetter) return null;
      }
      if (s.charAt(0) == '-') {
          sign = -1;
          s = s.substring(1);
      }
      var parts = s.split(/[\s°º˚′'’″"]+/).filter(function(p) { return p != ''; });
      if (parts.length < 1 || parts.length > 3) return null;
      for (var i = 0; i < parts.length; i++) {
          if (!/^\d+(\.\d+)?$/.test(parts[i])) return null;
      }
      var deg = parseFloat(parts[0]);
      var min = parts.length > 1 ? parseFloat(parts[1]) : 0;
      var sec = parts.length > 2 ? parseFloat(parts[2]) : 0;
      if (min >= 60 || sec >= 60) return null;
      var value = deg + min / 60 + sec / 3600;
      if (value > max) return null;
      return Math.round(sign * value * 1000000) / 1000000;
  }

  function showCoordinatePreview(field, positiveLetter, negativeLetter, max) {
      var text = $('#' + field).val();
      var value = parseCoordinate(text, positiveLetter, negativeLetter, max);
      var preview = $('#' + field + '-preview');
      if (String(text).trim() == '') preview.css('color', '').text('');
      else if (value == null) preview.css('color', '#c0392b').text('Not understood');
      else preview.css('color', '#198754').text('= ' + value);
  }

  $(document).on('input', '#latitude', function() { showCoordinatePreview('latitude', 'N', 'S', 90); });
  $(document).on('input', '#longitude', function() { showCoordinatePreview('longitude', 'E', 'W', 180); });

  $(document).on('click', '#save-gps-button', function() {
      var latText = $('#latitude').val();
      var longText = $('#longitude').val();
      if (latText.trim() == '') {
          alert("Please input the latitude");
          return false;
      }
      if (longText.trim() == '') {
          alert("Please input the longitude");
          return false;
      }
      var lat = parseCoordinate(latText, 'N', 'S', 90);
      var long = parseCoordinate(longText, 'E', 'W', 180);
      if (lat == null) {
          alert("The latitude is not understood. Use decimal (-33.859722) or degrees minutes seconds (33 51 35 S).");
          return false;
      }
      if (long == null) {
          alert("The longitude is not understood. Use decimal (151.211111) or degrees minutes seconds (151 12 40 E).");
          return false;
      }
    $.ajax({
      type: "POST",
      url: "/DaffodilServlet",
      data:{formName:"SetGPS",lat:lat,long:long},
      success: function (result) {
          currentData = JSON.parse(result);
          refreshScreen();
      },
      error: function(data){
          console.log("solo:" + JSON.stringify(data));
          alert("There was an error processing your request:" + JSON.stringify(data));
          return false;
      }
  });

});

//
// CSW (config switch) calibration
//
// Per-device calibration of the config switch ladder's ADC reference reading - lost whenever
// the battery is replaced, since the raw reading scales with whatever the newly attached
// battery/boost-converter actually outputs (see cswReferenceRaw comment in
// DaffodilWifiManager.cpp / the CalibrateCSWReference serial command in Daffodil.ino). Web
// equivalent of running that serial command then resetting the device.
$(document).on('click','#calibrate-csw-button', function (){
    if(!confirm("Set ALL config switches to OFF (00000, no solar) first, then press OK.\n\nThe device will reboot immediately to capture the calibration reading.")){
        return false;
    }
    $.ajax({
        type: "POST",
        url: "/DaffodilServlet",
        data:{formName:"CalibrateCSW"},
        success: function (result) {
        },
        error: function(data){
            // Expected: the device reboots before it can send a response, so this always
            // "fails" even on success - the reboot is the confirmation.
        }
    });
    alert("Calibration armed - device is rebooting now. Please wait about 10 seconds, then set the switches back to your normal operating mode and refresh this page.");
});

//
// sceptic
//
$(document).on('click','#daffodil-sceptic-config-icon', function (){
   $('.display-module').removeClass('d-block').addClass('d-none');
    $('#sceptic-configure').removeClass('d-none').addClass('d-block');
    $('#sceptic-max-field').val(currentData.troughlevelmaximumcm);
    $('#sceptic-min-field').val(currentData.troughlevelminimumcm);
    $('#sceptic-height-field').val(currentData.maximumScepticHeight);
});

$(document).on('click','#set-sceptic-max-current', function (){
    $('#sceptic-max-field').val(currentData.measuredHeight);
});

$(document).on('click','#set-sceptic-min-current', function (){
     $('#sceptic-min-field').val(currentData.measuredHeight);
});

$(document).on('click','#sceptic-configure-cancel', function (){
   refreshScreen();
});

$(document).on('click','#sceptic-configure-submit', function (){
    var height =  parseFloat($('#sceptic-height-field').val());
   var max = parseFloat($('#sceptic-max-field').val());
   var min =  parseFloat($('#sceptic-min-field').val());

    if(max<5 || min<5){
        alert("Please input valid max and min");
        return false;
    }

    if(max< min){
        alert("Max has to be greater than min");
        return false;
    }

    if(height< max){
        alert("Max has to be smaller than the height");
        return false;
    }

   $.ajax({
        type: "POST",
        url: "/DaffodilServlet",
        data:{formName:"SetScepticRange",height:height,max:max,min:min},
        success: function (result) {
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: function(data){
            console.log("solo:" + JSON.stringify(data));
            alert("There was an error processing your request:" + JSON.stringify(data));
            return false;
        }
    });

 });


//
// end of sceptic
//

$(document).on('click','.show-raw-data', function (){
    var tabledata='<table class="table table-striped">';
    var icon;
    for(const key in  currentData ){
        if(key!='ssids'){
            tabledata+='<tr><td>'+key+'</td><td>'+currentData[key]+'</td></tr>';
        }else{
            var ssids = JSON.parse(currentData[key]);
            tabledata+='<tr><td>'+key+'</td><<td>';
            for(let i = 0; i < ssids.length; i++) {
                let s = ssids[i];
                if(s.secure==0)icon='';
                else icon='<i class="fa fa-lock"></i>';
                tabledata+=s.ssid + " rssi=" + s.rssi + " ch=" + s.channel + " " + icon + '<br>';
            }
             tabledata+='</td></tr>';
        }   
    }
    tabledata+='</table>';
    $('#raw-data-table').empty();
    $('#raw-data-table').append(tabledata);
    $('#raw-data-title').html(currentData.devicename);
    $('#raw-data-json').empty();
    $('#raw-data-json').append('<code>'+JSON.stringify(currentData) + '</code>');
    $('#show-raw-data-modal').modal('show');
 });

function refreshData(){
    $.ajax({
         type: "GET",
         async:false,
         url: "/DaffodilServlet",
         data:{formName:"GetWebData"},
         success: function (result) {
             currentData = JSON.parse(result);
             
         },
         error: function(data){
             console.log("solo:" + JSON.stringify(data));
             alert("There was an error processing your request:" + JSON.stringify(data));
             return false;
         }
     });
  }

function refreshScreen(){ 
    $('.display-module').removeClass('d-block').addClass('d-none');
    
    
    $("#wifi-config-icon").removeClass (function (index, className) {
        return (className.match (/\btext-\S+/g) || []).join(' ');
    });
    $('#host-name').html(currentData.devicename);
    $('#ssid-name').html(currentData.ssid);
    var d = new Date(currentData.secondsTime*1000);
    const minutes = String(d.getMinutes()).padStart(2, '0');
    const seconds = String(d.getSeconds()).padStart(2, '0');
    if(currentData.digitalStablesUpload){
        $('#ds-upload').css('color', 'green');
    }else{
        $('#ds-upload').css('color', 'red');
    }

    var datestring = d.getHours() + ":" + minutes+":"+ seconds + " " + d.getDate() +"/" +(d.getMonth()+1) + "/" + d.getFullYear();
    $('#time').html(datestring);
    
    if(currentData.lora){
        $('#lora-logo').css('color', 'blue');
    }else{
        $('#lora-logo').css('color', 'red');
    }
    
    $('#serial-number').html(currentData.serialnumber);
    $('#temperature').html( currentData.temperature + '&#8451;');

    $("#rtc-bat-volt").html((Math.round(currentData.rtcBatVolt * 100) / 100).toFixed(2));
    if(currentData.rtcBatVolt>=2.7){
        $('#rtc-battery-icon').css("color", "green");
    }else if(currentData.rtcBatVolt>=2.5 && currentData.rtcBatVolt<2.7){
        $('#rtc-battery-icon').css("color", "yellow");
    }else if(currentData.rtcBatVolt<2.5){
        $('#rtc-battery-icon').css("color", "red");
    }
    if(currentData.rssi>-60 && currentData.rssi<=-50){
        $('#wifi-config-icon').addClass('text-sucess');
    }else if(currentData.rssi>-80 && currentData.rssi<-60){
        $('#wifi-config-icon').addClass('text-warning');
    }else if( currentData.rssi<-80){
        $('#wifi-config-icon').addClass('text-danger');
    }

    $('#wifi-info').removeClass('d-none').addClass('d-block');

    if(currentData.currentFunctionValue==FUN_1_FLOW){
       showSensor1();
    }else if(currentData.currentFunctionValue==FUN_2_FLOW){
        showSensor1();
        showSensor2();
    }else if(currentData.currentFunctionValue==FUN_1_FLOW_1_TANK){
        showSensor1();
        showTank1();
    }else if(currentData.currentFunctionValue==FUN_1_TANK){
        showTank1();
    }else if(currentData.currentFunctionValue==FUN_2_TANK){
        showTank1();
        showTank2();
    }else if(currentData.currentFunctionValue==DAFFODIL_SCEPTIC_TANK){
        showDaffodil();
    }else if(currentData.currentFunctionValue==DAFFODIL_WATER_TROUGH || currentData.currentFunctionValue==DAFFODIL_WATER_TROUGH_WATER_TEMP){
        showDaffodil();
    }else if(currentData.currentFunctionValue==DAFFODIL_WATER_TROUGH_TANK1){
        // Trough half of this mode isn't wired up in firmware yet (shared TRIGGER/ECHO pins
        // collide with tank1's pressure wiring) - only tank1 is actually readable, see
        // Daffodil.ino's Known Issues. Still show the trough config panel so height/min/max can
        // be pre-set ahead of that wiring landing.
        showDaffodil();
        showTank1();
    }else if(currentData.currentFunctionValue==DAFFODIL_2_WATER_TROUGH){
        // 2nd trough sensor reading isn't implemented yet (pending hardware) - only trough 1's
        // existing config/display applies for now.
        showDaffodil();
    }else if(currentData.currentFunctionValue==DAFFODIL_VIEWER){
        showViewer();
    }
}

//
// Power panel (2026-10-14) - lightning button under the cog. GetPowerData is filled by
// Daffodil.ino fillPowerData(): live readings, the battery ledger's latest 70 ms sample, and the
// last VitalSignsRecord the device built.
//
var powerTimer = null;
var OP_STATUS_NAMES = {0: 'Unknown', 1: 'Pulse sleep', 2: 'No LED', 3: 'Full', 4: 'Cloudy', 5: 'Comma'};
var RESET_REASONS = {0: 'Unknown', 1: 'Power on', 2: 'External pin', 3: 'Software', 4: 'Crash', 5: 'Interrupt watchdog',
    6: 'Task watchdog', 7: 'Watchdog', 8: 'Deep sleep', 9: 'Brownout', 10: 'SDIO'};
// Same LiFePO4 voltage table as the Cerebellum's LedgerSoc (anchors), interpolated.
var LIFEPO4_SOC = [[3.65, 100], [3.50, 99], [3.45, 95], [3.40, 90], [3.35, 80], [3.30, 70], [3.25, 60], [3.20, 50],
    [3.15, 40], [3.10, 30], [3.05, 20], [3.00, 10], [2.90, 5], [2.80, 2], [2.50, 0]];

function socFromVoltage(v){
    if(v >= LIFEPO4_SOC[0][0]) return 100;
    for(var i = 1; i < LIFEPO4_SOC.length; i++){
        var hi = LIFEPO4_SOC[i - 1], lo = LIFEPO4_SOC[i];
        if(v >= lo[0]) return lo[1] + (v - lo[0]) / (hi[0] - lo[0]) * (hi[1] - lo[1]);
    }
    return 0;
}

function pwRow(label, value){
    return '<div class="pw-row"><span>' + label + '</span><span>' + value + '</span></div>';
}

function pwDuration(sec){
    sec = Math.round(sec);
    if(sec < 60) return sec + ' s';
    if(sec < 3600) return Math.floor(sec / 60) + ' min';
    if(sec < 86400) return Math.floor(sec / 3600) + ' h ' + Math.floor((sec % 3600) / 60) + ' min';
    return Math.floor(sec / 86400) + ' d ' + Math.floor((sec % 86400) / 3600) + ' h';
}

function pwCurrent(ma, positiveWord, negativeWord){
    var a = Math.abs(ma).toFixed(1) + ' mA';
    if(Math.abs(ma) < 1) return a;
    return a + ' ' + (ma > 0 ? positiveWord : negativeWord);
}

function showPowerInfo(d){
    var live = d.live || {};
    var l = d.ledger || {};
    var batteryV = l.batteryV || live.batteryV || 0;
    var batteryMa = l.batteryV ? l.batteryMa : live.batteryMa;
    var panelMa = l.batteryV ? l.panelMa : live.panelMa;
    var panelV = l.panelV || live.panelV || 0;

    // Battery picture: SOC from the voltage, green 50 %+, yellow 20-50 %, red below 20 %.
    var soc = Math.round(socFromVoltage(batteryV));
    var color = soc >= 50 ? '#2e7d32' : soc >= 20 ? '#f9c74f' : '#c62828';
    // The whole battery takes the colour; the number is black on yellow, white on green and red.
    $('#pw-battery-fill').css({width: '100%', background: color});
    $('#pw-battery-text').text(batteryV > 1 ? soc + ' %' : 'no battery').css('color', soc >= 20 && soc < 50 ? '#000' : '#fff');
    // While charging the charger lifts the terminal voltage, so the voltage table reads high
    // (91 % for a cell at about 65 %, 2026-10-14 on a power bank) - say so, and grey the battery.
    var charging = batteryMa < -5;
    $('#pw-battery-fill').css('opacity', charging ? 0.45 : 1);
    if(charging) $('#pw-battery-text').text(soc + ' %*').css('color', '#000');
    $('#pw-soc-note').html(batteryV <= 1 ? '' : charging
        ? '<span style="color:#ef6c00">* Charging - the voltage reads high, so this is too high. It settles 10-30 min after charging stops.</span>'
        : 'about ' + Math.round(soc / 100 * d.capacityMah) + ' of ' + d.capacityMah + ' mAh &middot; estimated from ' + batteryV.toFixed(3) + ' V');

    var html = '';
    html += pwRow('Battery', batteryV.toFixed(3) + ' V');
    html += pwRow('Battery current', pwCurrent(batteryMa, 'out', 'charging'));
    html += pwRow('Solar panel', panelV.toFixed(2) + ' V');
    html += pwRow('Solar panel current', Math.max(0, panelMa).toFixed(1) + ' mA');
    html += pwRow('V50_I (charger input)', (live.v50iV || 0).toFixed(2) + ' V');
    html += pwRow('Mode', OP_STATUS_NAMES[live.operatingStatus] || ('Mode ' + live.operatingStatus));
    html += pwRow('Weather forecast', live.weatherAgeSec >= 9999 ? 'never received' : (live.weatherAgeSec < 1860 ? 'fresh, ' : 'stale, ') + pwDuration(live.weatherAgeSec) + ' old');
    if(live.wifiOnDemandSec > 0) html += pwRow('WiFi on demand', pwDuration(live.wifiOnDemandSec) + ' left');
    html += pwRow('Awake for', pwDuration(d.uptimeSec));
    // WiFi's cost since the last reset: battery charge out while WiFi was on, and the average.
    var we = d.wifiEnergy;
    if(we && we.onSec > 0){
        html += pwRow('WiFi used', we.dischargeOutMah.toFixed(2) + ' mAh in ' + pwDuration(we.onSec)
            + ' (avg ' + (we.dischargeOutMah / (we.onSec / 3600)).toFixed(0) + ' mA)');
    }
    $('#pw-sensors').html(html);

    var v = d.vital;
    if(!v){
        $('#pw-vital').html('<div style="font-size:14px;color:#6c757d">Not built yet this wake - the first report goes out with the next LoRa send.</div>');
        return;
    }
    var coverage = v.awakeSec > 0 ? Math.min(100, v.ledgerMeasuredSec / v.awakeSec * 100).toFixed(1) + ' %' : '-';
    html = '';
    html += pwRow('Report built', pwDuration(v.ageSec) + ' ago (#' + v.seq + ')');
    html += pwRow('Resets', v.resetCount + ' - last ' + (RESET_REASONS[v.lastResetReason] || v.lastResetReason)
        + (v.lastResetAgoSec ? ', ' + pwDuration(v.lastResetAgoSec) + ' ago' : ''));
    html += pwRow('Wakes', v.wakeCount + (v.earlyWakeCount ? ' (' + v.earlyWakeCount + ' early)' : '') + (v.commaWakeCount ? ', ' + v.commaWakeCount + ' comma' : ''));
    html += pwRow('Awake / asleep', pwDuration(v.awakeSec) + ' / ' + pwDuration(v.sleptSec));
    html += pwRow('Wake voltage', v.wakeV ? v.wakeV.toFixed(3) + ' V' : '-');
    html += pwRow('Lowest voltage', v.minV ? v.minV.toFixed(3) + ' V' : '-');
    html += pwRow('Charge in', v.chargeInMah.toFixed(2) + ' mAh');
    html += pwRow('Charge out', v.dischargeOutMah.toFixed(2) + ' mAh');
    html += pwRow('Panel energy', v.panelInWh.toFixed(3) + ' Wh');
    html += pwRow('Ledger coverage', coverage + (v.ledgerMaxGapMs ? ', max gap ' + v.ledgerMaxGapMs + ' ms' : ''));
    html += pwRow('Last LoRa send', v.txMs + ' ms, peak ' + v.txPeakMa + ' mA' + (v.txMinV ? ', dip ' + v.txMinV.toFixed(3) + ' V' : ''));
    html += pwRow('LoRa failures', v.loraTxFails);
    html += pwRow('Firmware', v.firmwareBuild);
    $('#pw-vital').html(html);
}

function loadPowerInfo(){
    $.ajax({
        type: "GET",
        url: "/DaffodilServlet",
        data: {formName: "GetPowerData"},
        success: function(result){ showPowerInfo(JSON.parse(result)); },
        error: function(){ $('#pw-sensors').html('<div style="color:#c62828">Cannot reach the Daffodil</div>'); }
    });
}

function stopPowerInfo(){
    if(powerTimer){ clearInterval(powerTimer); powerTimer = null; }
}

$(document).on('click','#power-info-icon', function (){
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#power-info').removeClass('d-none').addClass('d-block');
    loadPowerInfo();
    stopPowerInfo();
    powerTimer = setInterval(loadPowerInfo, 5000);
});

$(document).on('click','#power-info-close', function (){
    stopPowerInfo();
    refreshScreen();
});

$(document).on('click','#system-info-icon', function (){
    stopPowerInfo();
});

//
// Viewer panel (switch 1001, 2026-11-06; devices dynamic 2026-11-07). The Viewer shows data heard
// over LoRa on the LEDs; this edits which pages, in what order. GetViewerConfig returns the config,
// the device types with their fields, and the devices heard so far - only those are offered, plus
// any device a saved page already uses. SetViewerConfig saves a config (format in ViewerConfig.h).
// Loaded once - refreshScreen() runs after every other form, and reloading would wipe unsaved edits.
//
var viewerTypes = {};    // t -> {name, fields:[{id,label,glyph}]}
var viewerDevices = [];  // [{t, d, ageSec, rssi}] heard, newest first
var viewerPages = [];    // [{t, d, f, s}]
var viewerLoaded = false;
var VIEWER_MAX_PAGES = 8;

function showViewer(){
    $('#viewer-info').removeClass('d-none').addClass('d-block');
    if(!viewerLoaded) loadViewer();
}

function loadViewer(){
    $('#viewer-status').text('Loading...');
    $.ajax({
        type: "GET",
        url: "/DaffodilServlet",
        data:{formName:"GetViewerConfig"},
        success: function (result) {
            renderViewer(JSON.parse(result));
            viewerLoaded = true;
            $('#viewer-status').text('');
        },
        error: function(data){
            $('#viewer-status').text('Could not load the viewer config (' + data.status + ')');
        }
    });
}

function viewerAge(sec){
    if(sec < 120) return sec + ' s ago';
    if(sec < 7200) return Math.round(sec/60) + ' min ago';
    return Math.round(sec/3600) + ' h ago';
}

function viewerField(t, id){
    var type = viewerTypes[t];
    if(!type) return null;
    for(var i=0;i<type.fields.length;i++) if(type.fields[i].id==id) return type.fields[i];
    return null;
}

function viewerKey(t, d){ return t + '|' + (d || ''); }

function viewerDeviceText(t, d){
    var type = viewerTypes[t];
    var typeName = type ? type.name : t;
    if(t == 'forecast') return typeName;
    if(!d) return 'Any ' + typeName;
    return d == typeName ? d : d + ' (' + typeName + ')';
}

function viewerHeardDevice(t, d){
    for(var i=0;i<viewerDevices.length;i++){
        var h = viewerDevices[i];
        if(h.t == t && (!d || h.d == d)) return h;
    }
    return null;
}

// The dropdown: devices heard, then any a page uses that hasn't been heard (yet).
function viewerDeviceOptions(){
    var list = [], seen = {};
    for(var i=0;i<viewerDevices.length;i++){
        var k = viewerKey(viewerDevices[i].t, viewerDevices[i].d);
        if(seen[k] || !viewerTypes[viewerDevices[i].t]) continue;
        seen[k] = true;
        list.push({t: viewerDevices[i].t, d: viewerDevices[i].d, text: viewerDeviceText(viewerDevices[i].t, viewerDevices[i].d)});
    }
    for(var i=0;i<viewerPages.length;i++){
        var p = viewerPages[i], k = viewerKey(p.t, p.d);
        if(seen[k]) continue;
        seen[k] = true;
        var text = viewerDeviceText(p.t, p.d);
        if(!viewerHeardDevice(p.t, p.d)) text += ' - not heard';
        list.push({t: p.t, d: p.d, text: text});
    }
    return list;
}

function renderViewer(v){
    if(!v.types){  // firmware before 2026-11-07 sends a fixed field list instead
        $('#viewer-heard').empty().append($('<div class="text-danger">').text('This page needs newer Daffodil firmware - flash the latest release.'));
        $('#viewer-pages').empty();
        $('#viewer-add-page').prop('disabled', true);
        return;
    }
    viewerTypes = {};
    var types = v.types || [];
    for(var i=0;i<types.length;i++) viewerTypes[types[i].t] = {name: types[i].name, fields: types[i].fields || []};
    viewerDevices = (v.devices || []).slice().sort(function(a, b){ return a.ageSec - b.ageSec; });
    viewerPages = [];
    var pages = (v.config && v.config.pages) || [];
    for(var i=0;i<pages.length;i++) viewerPages.push({t:pages[i].t, d:pages[i].d || '', f:pages[i].f, s:pages[i].s || 0});
    $('#viewer-stale').val(v.config ? v.config.stale : 600);
    $('#viewer-frost').val(v.config ? v.config.frost : 0);

    var heard = $('#viewer-heard').empty();
    if(viewerDevices.length == 0){
        heard.append($('<div class="text-danger">').text('Nothing heard over LoRa yet - devices appear here as they are heard.'));
    }else{
        heard.append($('<div>').text('Heard:'));
        for(var i=0;i<viewerDevices.length;i++){
            var h = viewerDevices[i];
            if(!viewerTypes[h.t]) continue;
            heard.append($('<div>').text('• ' + viewerDeviceText(h.t, h.d) + ' - ' + viewerAge(h.ageSec)
                + (h.rssi !== undefined ? ', ' + h.rssi + ' dBm' : '')));
        }
    }
    drawViewerPages();
}

function drawViewerPages(){
    // One card per page - a table didn't fit a phone (2026-11-06). Device first, then that
    // device's variables (2026-11-07).
    var options = viewerDeviceOptions();
    var box = $('#viewer-pages').empty();
    for(var i=0;i<viewerPages.length;i++){
        var p = viewerPages[i];
        var f = viewerField(p.t, p.f);
        var header = $('<div class="card-header d-flex align-items-center" style="padding:4px 8px">')
            .append($('<strong class="me-auto">').text('Page ' + (i + 1) + (p.t == 'forecast' ? ' (blinks)' : '')))
            .append($('<button type="button" class="btn btn-sm btn-outline-secondary viewer-up" title="Move up">&#8593;</button>').attr('data-i', i).prop('disabled', i == 0))
            .append($('<button type="button" class="btn btn-sm btn-outline-secondary viewer-down" style="margin-left:4px" title="Move down">&#8595;</button>').attr('data-i', i).prop('disabled', i == viewerPages.length - 1))
            .append($('<button type="button" class="btn btn-sm btn-outline-danger viewer-remove" style="margin-left:4px" title="Remove">&#215;</button>').attr('data-i', i).prop('disabled', viewerPages.length == 1));
        var device = $('<select class="form-select form-select-sm viewer-device" style="margin-bottom:6px">').attr('data-i', i);
        for(var j=0;j<options.length;j++) device.append($('<option>').val(viewerKey(options[j].t, options[j].d)).text(options[j].text));
        device.val(viewerKey(p.t, p.d));
        var field = $('<select class="form-select viewer-field">').attr('data-i', i);
        var type = viewerTypes[p.t];
        var fields = type ? type.fields : [];
        for(var j=0;j<fields.length;j++) field.append($('<option>').val(fields[j].id).text(fields[j].label));
        field.val(p.f);
        var body = $('<div class="card-body" style="padding:8px">').append(device, field);
        // Temperature digits use LEDs 4 and 14 themselves, so those pages get no dot.
        if(!(f && f.glyph == 'temperature')){
            var dot = $('<select class="form-select form-select-sm viewer-dot" style="width:auto;display:inline-block;margin-left:6px">').attr('data-i', i);
            dot.append($('<option>').val(0).text('none'));
            dot.append($('<option>').val(1).text('LED 4'));
            dot.append($('<option>').val(2).text('LED 14'));
            dot.val(p.s);
            body.append($('<div class="small" style="margin-top:6px">').text('Slot dot').append(dot));
        }
        box.append($('<div class="card" style="margin-bottom:8px">').append(header, body));
    }
    $('#viewer-add-page').prop('disabled', viewerPages.length >= VIEWER_MAX_PAGES || options.length == 0);
}

// A page on device t/d showing its first field (or keeping f if that device type has it).
function viewerSetDevice(page, t, d){
    var type = viewerTypes[t];
    page.t = t;
    page.d = t == 'forecast' ? '' : d;
    if(!viewerField(t, page.f)) page.f = type && type.fields.length ? type.fields[0].id : '';
    var f = viewerField(t, page.f);
    if(f && f.glyph == 'temperature') page.s = 0;
}

$(document).on('change', '.viewer-device', function (){
    var i = parseInt($(this).attr('data-i'));
    var k = $(this).val(), bar = k.indexOf('|');
    viewerSetDevice(viewerPages[i], k.substring(0, bar), k.substring(bar + 1));
    drawViewerPages();
});

$(document).on('change', '.viewer-field', function (){
    var i = parseInt($(this).attr('data-i'));
    viewerPages[i].f = $(this).val();
    var f = viewerField(viewerPages[i].t, viewerPages[i].f);
    if(f && f.glyph == 'temperature') viewerPages[i].s = 0;
    drawViewerPages();
});

$(document).on('change', '.viewer-dot', function (){
    viewerPages[parseInt($(this).attr('data-i'))].s = parseInt($(this).val());
});

$(document).on('click', '.viewer-up, .viewer-down', function (){
    var i = parseInt($(this).attr('data-i'));
    var j = $(this).hasClass('viewer-up') ? i - 1 : i + 1;
    var t = viewerPages[i]; viewerPages[i] = viewerPages[j]; viewerPages[j] = t;
    drawViewerPages();
});

$(document).on('click', '.viewer-remove', function (){
    viewerPages.splice(parseInt($(this).attr('data-i')), 1);
    drawViewerPages();
});

$(document).on('click', '#viewer-add-page', function (){
    var options = viewerDeviceOptions();
    if(viewerPages.length >= VIEWER_MAX_PAGES || options.length == 0) return;
    var page = {t:'', d:'', f:'', s:0};
    viewerSetDevice(page, options[0].t, options[0].d);  // the device heard most recently
    viewerPages.push(page);
    drawViewerPages();
});

$(document).on('click', '#viewer-save', function (){
    var stale = parseInt($('#viewer-stale').val());
    var frost = parseFloat($('#viewer-frost').val());
    if(isNaN(stale) || stale < 30 || stale > 65535){ alert('"No data after" must be 30 to 65535 seconds'); return; }
    if(isNaN(frost)){ alert('Frost threshold must be a number'); return; }
    var pages = [];
    for(var i=0;i<viewerPages.length;i++){
        var v = viewerPages[i];
        var p = {t: v.t, f: v.f};
        if(v.d) p.d = v.d;
        if(v.s) p.s = v.s;
        pages.push(p);
    }
    var config = {stale: stale, frost: frost, pages: pages};
    $('#viewer-status').text('Saving...');
    $.ajax({
        type: "POST",
        url: "/DaffodilServlet",
        data:{formName:"SetViewerConfig", config: JSON.stringify(config)},
        success: function (result) {
            renderViewer(JSON.parse(result));
            $('#viewer-status').text('Saved - the LEDs use it from the next page.');
        },
        error: function(data){
            $('#viewer-status').text('Not saved: ' + (data.responseText || data.status));
        }
    });
});
