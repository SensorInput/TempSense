--Webserver starten 
print("Webserver startet")

-- LED an PIN 4 --> GPIO2 initialisieren
gpio.mode(4, gpio.OUTPUT)

-- Let's start off with the light off
gpio.write(4, gpio.LOW)

print("WIFI Status vor Serverstart, WIFI Status "..wifi.sta.status())

   
srv=net.createServer(net.TCP, 3)


print("Server created on " .. wifi.sta.getip())
srv:listen(80,function(conn)
    conn:on("receive",function(conn,request)
        print(request)
        _, j = string.find(request, 'control_switch=')
        if j ~= nil then
            command = string.sub(request, j + 1)
                if command == 'on' then
                    gpio.write(4, gpio.HIGH)
                else
                    gpio.write(4, gpio.LOW)
                end
        end
        conn:send('<!DOCTYPE html>')
        conn:send('<html lang="de">')
        conn:send('<head><meta charset="utf-8" />')
        conn:send('<title>ESP Server</title></head>')
        conn:send('<body><h1>Fan Control Web Interface</h1>')
    -- Send the current status of the Fan Control unit
        if gpio.read(4) == gpio.HIGH then
            fan = "ON"
        else
            fan = "OFF"
        end
        conn:send('<p> Status der Lueftereinheit : ' .. fan .. '</p>')
        conn:send('<form method="post">')
        conn:send('<input type="radio" name="control_switch" value="on">ON</input><br />')
        conn:send('<input type="radio" name="control_switch" value="off">OFF</input><br />')
        conn:send('<input type="submit" value="Fan Control Switch" />')
        conn:send('</form></body></html>')
    end)
end)
