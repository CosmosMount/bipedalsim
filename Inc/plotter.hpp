#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

class Plotter 
{
public:
    Plotter(const std::string& ip = "127.0.0.1", int port = 9870) 
    {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        servaddr.sin_addr.s_addr = inet_addr(ip.c_str());
    }
    ~Plotter() { close(sockfd); }

    void send(const std::string& data) 
    {
        sendto(sockfd, data.c_str(), data.size(), 0, (struct sockaddr*)&servaddr, sizeof(servaddr));
    }

private:
    int sockfd;
    struct sockaddr_in servaddr;
};