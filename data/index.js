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
        url: "/RosieServlet",
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
            url: "/RosieServlet",
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
            url: "/RosieServlet",
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
        type: "GET",
        url: "/RosieServlet",
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
        type: "GET",
        url: "/RosieServlet",
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
   if(tankheightmeters==0){
    alert('Tank height has to be greater than zero');
    return false;
   }
   if(tank1maxvollit==0){
    alert('Tank volume has to be greater than zero');
    return false;
   }
   $.ajax({
        type: "GET",
        url: "/RosieServlet",
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
        type: "GET",
        url: "/RosieServlet",
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
    $('#tank-sensor-1-value').html(currentData.tank1waterLevel);
    $('#tank-sensor-1-units').html('liters');
    $('#tank-sensor-1').removeClass('d-none').addClass('d-block');
 }

function showTank2(){
    $('#tank-sensor-2-name').html(currentData.tank2name);
    $('#tank-sensor-2-value').html(currentData.tank2waterLevel);
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
    $('#daffodil-capacitor-value').html(currentData.capacitorVoltage.toFixed(2));  
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
         url: "/RosieServlet",
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
         url: "/RosieServlet",
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

  $(document).on('click','#save-gps-button', function (){
    var lat = $('#latitude').val();
    var long = $('#longitude').val();
    if(lat==''){
        alert("Please input the latitude");
        return false;
    }
    if(long==''){
        alert("Please input the longitude");
        return false;
    }
    $.ajax({
      type: "POST",
      url: "/RosieServlet",
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
    }else if(currentData.currentFunctionValue==DAFFODIL_SCEPTIC_TANK){
        showDaffodil();
    }else if(currentData.currentFunctionValue==DAFFODIL_WATER_TROUGH){
        showDaffodil();
    }
}