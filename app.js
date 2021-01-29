const app = require('http').createServer(handler);
const fs = require('fs');
const os = require('os');
const io = require('socket.io')(app);
const SerialPort = require("serialport");
const Readline = require('@serialport/parser-readline')


const port = new SerialPort('/dev/tty.usbmodem14101', {
    baudRate: 38400
    }, function (err) {
    if (err) {
        return console.log('Error on open: ', err.message)
    } else {
        return console.log('Connected to serial port')
    }
});


const parser = port.pipe(new Readline({ delimiter: '\r\n' }))
parser.on('data', function(data) {
    // console.log('Data: ', data);
    io.sockets.emit('position_update', data);
});

app.listen(5000);

// Http handler function
function handler (req, res) {
    
    var path = req.url;
    console.log(path)
    
    // Managing the root route
    if (path == '/') {
        index = fs.readFile(__dirname+'/client.html', 
            function(error,data) {
                if (error) {
                    res.writeHead(500);
                    return res.end("Error: unable to load client.html");
                }
                res.writeHead(200,{'Content-Type': 'text/html'});
                res.end(data);
            });

    // Managing the route for the javascript files
    } else if( /\.(js)$/.test(path) ) {
        index = fs.readFile(__dirname+path, 
            function(error,data) {
                
                if (error) {
                    res.writeHead(500);
                    return res.end("Error: unable to load " + path);
                }
                
                res.writeHead(200,{'Content-Type': 'text/plain'});
                res.end(data);
            });
    } else {
        res.writeHead(404);
        res.end("Error: 404 - File not found.");
    }
    
}