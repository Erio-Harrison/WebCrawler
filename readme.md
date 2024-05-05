# Gopher Client Project

## Overview
This Gopher Client is designed to interact with Gopher servers, retrieving and displaying content from specified Gopher directories. It supports both Windows and Unix-like systems, utilizing socket programming for network communication.

## Features
Connect to Gopher servers using TCP/IP sockets.

Navigate Gopher directories and retrieve text and binary files.

Multi-platform support with specific configurations for Windows and Unix-based systems.

## Requirements
C++ compiler (g++, Visual Studio, etc.)

Windows: Winsock2 library

Unix/Linux: Standard networking libraries (part of the system)

## How to use
### Compilation:
`Windows`: 

```bash
g++ -o main.exe main.cpp -lws2_32
```

`Unix/Linux`:
```bash
g++ -o main main.cpp
```

### Usage
`Windows`: 

```bash
.\main.exe
```

`Unix/Linux`:
```bash
./main
```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
