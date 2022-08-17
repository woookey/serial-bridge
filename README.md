# serial-bridge

## Introduction
Bridge between serial and UDP communication. Receives data from microcontroller or any other device
over serial port, packs it into JSON format and sends over UDP to the server.

## Compile and use
```
mkdir build && cd build
cmake .. && make install
./bin/serial_bridge 
```

## Contribute
serial-bridge is open to contribution to expand the functionality. You can check the current list 
of issues or suggest any new feature that would solve your problem with this tool.


## Copyright 
```
MIT License

Copyright (c) 2022 Actuated Robots Ltd., Lukasz Barczyk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```