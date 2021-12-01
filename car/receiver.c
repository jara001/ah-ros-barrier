#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

#define PORT 8080
#define SA struct sockaddr

struct {
    int64_t lap_time_us;
    int64_t best_time_us;
} state;


int main()
{
    int sockfd, connfd, len;
    struct sockaddr_in servaddr, cli;
   
    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        printf("socket creation failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully created..\n");
    bzero(&servaddr, sizeof(servaddr));
   
    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("10.37.1.117"); // ipv4 of the barrier
    servaddr.sin_port = htons(PORT);
   
    if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
        printf("connection with the server failed...\n");
        exit(0);
    }
    else
        printf("connected to the server..\n");

    int incom;

    while(1){
        incom = recv(sockfd, &state, sizeof(state), 0);
        if (incom > 0) 
            printf("%ld\t %ld\n", state.lap_time_us, state.best_time_us);
    }

    // for (int i = 0; i < 3; i++){
    //     incom = recv(sockfd, &state, sizeof(state), 0);
    //     if (incom > 0) 
    //         printf("%ld\t %ld\n", state.lap_time_us, state.best_time_us);
    // }
    close(sockfd);
}