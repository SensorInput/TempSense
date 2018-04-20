-- WIFI Verbindung herstellen

function checkWiFiStatus()
  local s = wifi.sta.status()
  print("WiFi Status: " .. s) 
  if s == 5 then
    tmr.stop(0)
    print("Connected on " .. wifi.sta.getip())
    print("Testausgabe Check WIFI")
    dofile("runWebServer.lua")
  end
end


wifi.setmode(wifi.STATION)
station_cfg={}
station_cfg.ssid="WIFISSID"
station_cfg.pwd="password" 
wifi.sta.config(station_cfg)
print('Verbindungsversuch')
wifi.sta.connect()

tmr.alarm(0, 1000, 1, checkWiFiStatus)
