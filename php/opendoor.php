<?php
function GetIP(){
    if(!empty($_SERVER["HTTP_CLIENT_IP"])){
      $cip = $_SERVER["HTTP_CLIENT_IP"];
    }
    elseif(!empty($_SERVER["HTTP_X_FORWARDED_FOR"])){
      $cip = $_SERVER["HTTP_X_FORWARDED_FOR"];
    }
    elseif(!empty($_SERVER["REMOTE_ADDR"])){
      $cip = $_SERVER["REMOTE_ADDR"];
    }
    else{
      $cip = "无法获取！";
    }
    return $cip;
}


$ip = GetIP();
//var_dump($_SERVER);
echo "您的IP地址是：".$ip;
if($ip === "59.67.152.241"){
    echo "open door"
    exec("sudo ./py_opendoor_client.py");
}else {
    echo "wrong code : 10000";
}


 ?>