# Portable CANaerospace library - libcanaerospace
This library implements the [CANaerospace 1.7 protocol][1] for Linux and embedded platforms.
It's mainly targeted for UAV systems.

### Features
- Clean API
- Hardware-agnostic (CAN drivers are decoupled from the library logic)
- Built-in support for redundant CAN bus
- Built-in support for redundant units (Redundancy channels)

### Supported hardware
Currently the following CAN implementations are supported out of the box:

- Linux SocketCAN
- STM32 embedded bxCAN

Other drivers can be interfaced with the library easily, refer to the examples to learn how.

# How to use
### On Linux
Build libcanaerospace:

    cd canaerospace
    mkdir build && cd build && cmake ..
    make
    make install
    # To run unit tests (gtest required):
    make tests

Build the SocketCAN driver (it's just a tiny static library implemented in few lines of C):

    cd drivers/socketcan
    make
    make install

Build the example too, why not:

    cd examples/linux
    mkdir build && cd build && cmake ..
    make

### On embedded system
Refer to [the relevant examples][2].

### Quick start
Consider the examples for a quick start:

- Linux example in this repository
- [Embedded examples are separated][2]

Also, you should be familiar with the [CANaerospace specification][3].

# TODO

- Automatic configuration of CAN filters

[1]: http://en.wikipedia.org/wiki/CANaerospace
[2]: https://bitbucket.org/pavel_kirienko/canaerospace_embedded_examples
[3]: http://www.stockflightsystems.com/tl_files/downloads/canaerospace/canas_17.pdf
