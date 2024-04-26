#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <cstring>
#include <chrono>
#include <iomanip>

#if defined(_WIN32) || defined(_WIN64)
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    #define CLOSESOCKET closesocket
    #define SOCKETTYPE SOCKET
#else
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netdb.h>
    #include <unistd.h> // For close function
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define CLOSESOCKET close
    #define SOCKETTYPE int
#endif

void initSockets() {
#if defined(_WIN32) || defined(_WIN64)
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed.\n";
    }
#else
    // No initialization needed for Unix/Linux
#endif
}

void cleanupSockets() {
#if defined(_WIN32) || defined(_WIN64)
    WSACleanup();
#else
    // No cleanup needed for Unix/Linux
#endif
}

void printLastError(const char* msg) {
#if defined(_WIN32) || defined(_WIN64)
    std::cerr << msg << ": " << WSAGetLastError() << std::endl;
#else
    perror(msg);
#endif
}

std::vector<std::string> gopherDirectories;
std::unordered_set<std::string> visitedDirectories;
std::unordered_map<std::string, size_t> textFiles;
std::unordered_map<std::string, size_t> binaryFiles;
std::map<std::string, bool> externalServers;
std::set<std::string> uniqueInvalidRefs;

std::string smallestTextFileContent;
size_t smallestTextFileSize = std::numeric_limits<size_t>::max();
size_t largestTextFileSize = 0;
size_t smallestBinaryFileSize = std::numeric_limits<size_t>::max();
size_t largestBinaryFileSize = 0;

// Function to send a line to a socket
void writeLine(SOCKETTYPE sock, const std::string& txt) {
    int length = txt.length();
    const char* buffer = txt.c_str();
    while (length > 0) {
        //In the context of TCP socket programming,
        //send function returns an integer value,
        //and iSendResult is used to capture this value.
        int iSendResult = send(sock, buffer, length, 0);
        if (iSendResult == SOCKET_ERROR) {
            printLastError("send failed with error");
            CLOSESOCKET(sock);
            #if defined(_WIN32) || defined(_WIN64)
                WSACleanup();
            #endif
            return;
        }
        length -= iSendResult;
        buffer += iSendResult;
    }
}

// Function to read a line from a socket
std::string readLine(SOCKETTYPE sock) {
    std::string line;
    char prev_char = '\0';
    char buffer;
    while (true) {
        int bytesReceived = recv(sock, &buffer, 1, 0);
        if (bytesReceived == 1) {
            if (buffer == '\n' && prev_char == '\r') {
                line.pop_back(); // Remove '\r' from the end of the line
                break;
            }
            line += buffer;
            prev_char = buffer;
        } else if (bytesReceived == 0) {
            // Connection closing
            break;
        } else {
            // Error occurred
            printLastError("Unable to connect to server");
            CLOSESOCKET(sock);
            #if defined(_WIN32) || defined(_WIN64)
                WSACleanup();
            #endif
            return "";
        }
    }
    return line;
}


const char* serviceHost = "comp3310.ddns.net";
int servicePort = 70;

struct GopherTask{
    std::string host;
    int port;
    std::string selector;

    GopherTask(std::string h, int p, std::string s) : host(std::move(h)), port(p), selector(std::move(s)) {}
};



// Global task queue
std::queue<GopherTask> taskQueue;

SOCKETTYPE connectToServer(const char* host, const char* portStr) {
    struct addrinfo hints, *result;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;  // IPV4
    hints.ai_socktype = SOCK_STREAM;  // TCP socket

    // Resolve the server address and port
    int iResult = getaddrinfo(host, portStr, &hints, &result);
    if (iResult != 0) {
        std::cerr << "getaddrinfo failed " << gai_strerror(iResult) << std::endl;
        return INVALID_SOCKET;
    }

    SOCKETTYPE sock = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (sock == INVALID_SOCKET) {
        printLastError("Error at socket()");
        freeaddrinfo(result);
        return INVALID_SOCKET;
    }

    iResult = connect(sock, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        CLOSESOCKET(sock);

        printLastError("Unable to connect to server");
        sock = INVALID_SOCKET;
    }

    freeaddrinfo(result);
    return sock;
}

size_t getFileSize(const std::string& selector, const std::string& host, int port) {
    size_t fileSize = 0;
    #if defined(_WIN32) || defined(_WIN64)
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "WSAStartup failed." << std::endl;
            return -1;  // Signal an error
        }
    #endif

    SOCKETTYPE sock = connectToServer(host.c_str(), std::to_string(port).c_str());
    if (sock == INVALID_SOCKET) {
        #if defined(_WIN32) || defined(_WIN64)
            WSACleanup;
        #endif
        return -1;  // Signal an error
    }

    std::string request = selector + "\r\n";
    writeLine(sock, request);

    char buffer[1024];
    int bytesReceived;
    const size_t maxDataLimit = 5 * 1024 * 1024;  // Limit to 5 MB to prevent firehose data overflow
    auto startTime = std::chrono::high_resolution_clock::now();
    const int timeoutSeconds = 5;  // 5 seconds timeout

    while ((bytesReceived = recv(sock, buffer, sizeof(buffer), 0)) > 0) {
        fileSize += bytesReceived;
        if (fileSize > maxDataLimit) {
            std::cerr << "Data limit exceeded, stopping reception." << std::endl;
            fileSize = -2;  // Indicate a special condition of data overflow
            break;
        }
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
        if (elapsed.count() >= timeoutSeconds) {
            std::cerr << "Timeout reached, stopping reception." << std::endl;
            fileSize = -1;  // Timeout error
            break;
        }
    }

    if (bytesReceived < 0) {
        printLastError("Reception failed");
        fileSize = -1;  // General error
    }

    CLOSESOCKET(sock);
    #if defined(_WIN32) || defined(_WIN64)
        WSACleanup();
    #endif
    return fileSize;
}

bool checkServerAvailability(const std::string& host, int port) {
    struct sockaddr_in serv_addr;
    SOCKETTYPE sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) return false;

    struct hostent *server = gethostbyname(host.c_str());
    if (server == NULL) {
        CLOSESOCKET(sockfd);
        return false;
    }

    memset(&serv_addr, 0, sizeof(serv_addr));  // Clear structure
    serv_addr.sin_family = AF_INET;
    memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);  // Copy the host address
    serv_addr.sin_port = htons(port);  // Convert the port number to network byte order

    int connect_result = connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    CLOSESOCKET(sockfd);

    return connect_result >= 0;
}

void processGopherLine(const std::string& line, const std::string& currentHost, int currentPort) {
    if (line.empty() || line == ".") return;

    char itemType = line[0];
    std::istringstream iss(line.substr(1));
    std::string description, selector, host;
    int port;
    std::string portStr;

    // Attempt to parse the line, using the existing host and port as defaults if missing
    bool validLine = std::getline(iss, description, '\t') &&
                     std::getline(iss, selector, '\t') &&
                     std::getline(iss, host, '\t');

    if (!std::getline(iss, portStr) || host.empty()) {
        host = currentHost; // Default to the current host if missing
        portStr = std::to_string(currentPort); // Default to the current port if missing
    }

    try {
        port = std::stoi(portStr);
    } catch (const std::invalid_argument& ia) {
        // Handle cases where port is non-numeric or missing
        port = currentPort; // Default to the current port
    }

    std::string fullPath = host + ":" + portStr + selector;
    if (itemType == '3') {  // Handle error items
        uniqueInvalidRefs.insert(fullPath);
        std::cerr << "Error item encountered at: " << fullPath << " - " << description << std::endl;
        return;  // Do not proceed with size fetching for error items
    }
    if (host == currentHost && port == currentPort) {
        if (selector == "/misc/godot") {
            std::cerr << "Skipping known problematic endpoint: " << fullPath << std::endl;
            return;  // Avoid even attempting to get the file size
        }
        size_t fileSize = getFileSize(selector, host, port);
        if (fileSize == static_cast<size_t>(-1)) {
            // Handle errors specifically, perhaps log them or store as an invalid reference.
            return; // Don't store these file sizes and return.
        }

        switch (itemType) {
            case '1': // Directory
                if (visitedDirectories.insert(fullPath).second) {
                    gopherDirectories.push_back(fullPath);
                    taskQueue.push(GopherTask(host, port, selector));
                }
                break;
            case '0': // Text file
                textFiles[fullPath] = fileSize;
                break;
            case '9': // Binary file
                binaryFiles[fullPath] = fileSize;
                break;
            case 'i': // Info item, ignore
                break;
            default:
                std::cerr << "Unhandled Gopher item type: '" << itemType << "' in line: " << line << std::endl;
        }
    } else {
        std::string serverKey = host + ":" + std::to_string(port);
        if (externalServers.find(serverKey) == externalServers.end()) {
            bool isUp = checkServerAvailability(host, port);
            externalServers[serverKey] = isUp;
        }
    }
}



void crawlGopherDirectory(SOCKETTYPE sock, const std::string& selector) {
    // Basic example of reading a line from the server
    std::string line = readLine(sock);
    while (!line.empty()) {
        // Process the line
        processGopherLine(line,serviceHost,servicePort);
        // Read the next line
        line = readLine(sock);
    }
}

void sendGopherRequest(SOCKETTYPE sock, const std::string& request) {
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_tm = *std::localtime(&now_time_t);

    // Format and print timestamp and request
    std::cout << "[" << std::put_time(&now_tm, "%H:%M:%S") << "] Sending request: " << request << std::endl;

    std::string formatted_request = request + "\r\n";
    writeLine(sock, formatted_request);
}

void inputLoop() {
    while (!taskQueue.empty()) {
        GopherTask task = taskQueue.front();
        taskQueue.pop();
        #if defined(_WIN32) || defined (_WIN64)
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
            std::cerr << "WSAStartup failed" << std::endl;
            continue;
        }
        #endif

        SOCKETTYPE sock = connectToServer(task.host.c_str(), std::to_string(task.port).c_str());
        if (sock != INVALID_SOCKET) {
            sendGopherRequest(sock, task.selector);
            crawlGopherDirectory(sock, task.selector);
            CLOSESOCKET(sock);
        }

        #if defined(_WIN32) || (_WIN64)
        WSACleanup();
        #endif
    }
}

std::string readFileContent(const std::string& fullPath) {
    // Extract host, port, and selector from fullPath
    std::istringstream iss(fullPath);
    std::string host, portStr, selector;
    getline(iss, host, ':');
    getline(iss, portStr, ':');
    getline(iss, selector);

    int port = std::stoi(portStr);
    std::string content;

    #if defined(_WIN32) || defined(_WIN64)
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            std::cerr << "WSAStartup failed." << std::endl;
            return "";
        }
    #endif

    SOCKETTYPE sock = connectToServer(host.c_str(), portStr.c_str());
    if (sock == INVALID_SOCKET) {
        #if defined(_WIN32) || defined(_WIN64)
            WSACleanup();
        #endif
        return "";
    }

    // Send the request
    std::string request = selector + "\r\n";
    writeLine(sock, request);

    // Read the response
    char buffer[1024];
    int bytesReceived;
    while ((bytesReceived = recv(sock, buffer, sizeof(buffer), 0)) > 0) {
        content.append(buffer, bytesReceived);
    }

    // Handle possible errors during reception
    if (bytesReceived < 0) {
        std::cerr << "Failed to read file content." << std::endl;
        content = ""; // Optional: Handle this case as needed
    }

    // Clean up
    CLOSESOCKET(sock);
    #if defined(_WIN32) || defined(_WIN64)
        WSACleanup();
    #endif

    return content;
}


void computeFinalDetails() {
    // Iterate over text files to find the smallest and largest files
    for (const auto& file : textFiles) {
        if (file.second < smallestTextFileSize) {
            smallestTextFileSize = file.second;
            // Fetch and update content for the smallest text file
            smallestTextFileContent = readFileContent(file.first);
        }
        if (file.second > largestTextFileSize) {
            largestTextFileSize = file.second;
        }
    }
    // Similarly, update smallest and largest sizes for binary files
    for (const auto& file : binaryFiles) {
        if (file.second < smallestBinaryFileSize) {
            smallestBinaryFileSize = file.second;
        }
        if (file.second > largestBinaryFileSize) {
            largestBinaryFileSize = file.second;
        }
    }
}



void printResults() {
    std::cout << "Number of Gopher directories: " << gopherDirectories.size() << std::endl;
    std::cout << "Number of text files: " << textFiles.size() << std::endl;
    for (const auto& file : textFiles) {
        std::cout << "Text file: " << file.first << " (Size: " << file.second << ")" << std::endl;
    }
    std::cout << "Number of binary files: " << binaryFiles.size() << std::endl;
    for (const auto& file : binaryFiles) {
        std::cout << "Binary file: " << file.first << " (Size: " << file.second << ")" << std::endl;
    }
    std::cout << "Smallest text file size: " << smallestTextFileSize << " Content: " << smallestTextFileContent << std::endl;

    std::cout << "Largest text file size: " << largestTextFileSize << std::endl;
    std::cout << "Smallest binary file size: " << smallestBinaryFileSize << std::endl;
    std::cout << "Largest binary file size: " << largestBinaryFileSize << std::endl;
    std::cout << "Number of unique invalid references: " << uniqueInvalidRefs.size() << std::endl;
    for (const auto& server : externalServers) {
        std::cout << "External server: " << server.first << " is " << (server.second ? "up" : "down") << std::endl;
    }
}

int main(int argc, char* argv[]) {
    initSockets();

    // Initialization
    if (argc > 1) {
        serviceHost = argv[1];
        if (argc > 2) {
            servicePort = std::stoi(argv[2]);
        }
    }


    // Seed the initial task
    taskQueue.push(GopherTask(serviceHost, servicePort, ""));
    inputLoop();  // Start processing the tasks
    computeFinalDetails();
    printResults();
    cleanupSockets();
    std::cout << "Done." << std::endl;

    return 0;
}