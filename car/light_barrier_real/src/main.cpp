/* His node takes 


*/



#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <csignal>
#include <iostream>

#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Int64MultiArray.h>

#define PORT 8080
#define SA struct sockaddr

using namespace std;

struct {
    int64_t lap_time_us;
    int64_t best_time_us;
} state;

// Our own Signal Handler so functions connect and revc are killed by CTRL+C 
void signalHandler( int signum ) {
   cout << "Interrupt signal (" << signum << ") received.\n";

   // cleanup and close up stuff here  
   // terminate program  

   ros::shutdown();
   exit(signum);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "light_barrier_real", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    signal(SIGINT, signalHandler);

    ros::Publisher time_pub = n.advertise<std_msgs::Int64MultiArray>("/light_barier_real", 1);

    int sockfd, connfd, len;
    struct sockaddr_in servaddr, cli;
   
    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        // printf("socket creation failed...\n");
        std::cout << "Socket creation failed..." << "\n";
        ros::shutdown();
    }
    else
        // printf("Socket successfully created..\n");
        std::cout << "Socket successfully created..." << "\n";
    bzero(&servaddr, sizeof(servaddr));
   
    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("10.37.1.117"); // ipv4 of the barrier
    servaddr.sin_port = htons(PORT);

    // std::cout << "Waiting for connection" << "\n";
    if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
        // printf("connection with the server failed...\n");
        std::cout << "Connection with the server failed..." << "\n";
        ros::shutdown();
    }
    else
        // printf("connected to the server..\n");
        std::cout << "Connected to the server..." << "\n";

    int incom;

    while(ros::ok()){

        std_msgs::Int64MultiArray msg;
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
        msg.layout.dim[0].label = "Lap time";
        msg.data.clear();

        // std::cout << "Waiting for message" << "\n";
        incom = recv(sockfd, &state, sizeof(state), 0);
        if (incom > 0) {
            msg.data.push_back(state.lap_time_us);
            msg.data.push_back(state.best_time_us);
            time_pub.publish(msg);
            // std::cout << "Message published" << "\n";
        }
            // printf("%ld\t %ld\n", state.lap_time_us, state.best_time_us);

    }

    // for (int i = 0; i < 3; i++){
    //     incom = recv(sockfd, &state, sizeof(state), 0);
    //     if (incom > 0) 
    //         printf("%ld\t %ld\n", state.lap_time_us, state.best_time_us);
    // }
    close(sockfd);
}