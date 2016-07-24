# cubes-opengl
Simple OpenGL demo with multiplayer feature. Player controls single cube with
mechanics inspired by "Katamari Damacy".

Details:
* Internally demo consist of client and server
* Both communicate over UDP using Snapshots Interpolation model*
* Server dumps simulation state 10 times per second (10pps)
* State is bound and quantizied before sending
* Client renders recived data and sends user input to server (smilar to Quake 2
  model)

## Build
Works on Windows and Linux :)
```sh
git clone https://github.com/dzeromsk/cubes-openg.git
mkdir cubes-openg/build
cd cubes-openg/build
cmake -GNinja ..
ninja
```
## Examples
### Start screen
You can connect to existing server or start new one.

![Alt text](doc/cube1.png?raw=true)

### Player view
Player controls large cube using WSAD.

![Alt text](doc/cube2.png?raw=true)

### Multiplayer
Player count is limited only by server troughput.

![Alt text](doc/cube3.png?raw=true)

### Debug view
Press F12 to open second udp connection and recive raw data (no interpolation 
and quantization) from server.

![Alt text](doc/cube4.png?raw=true)

# References
* http://gafferongames.com/networked-physics/the-physics-simulation/
* http://gafferongames.com/networked-physics/snapshots-and-interpolation/
* http://number-none.com/product/Packing%20Integers/
* http://number-none.com/product/Scalar%20Quantization/
* http://number-none.com/product/Transmitting%20Vectors/
* http://number-none.com/product/Adaptive%20Compression%20Across%20Unreliable%20Networks/
* http://number-none.com/product/Piecewise%20Linear%20Data%20Models/
* http://fabiensanglard.net/quake2/
* http://fabiensanglard.net/quakeSource/
