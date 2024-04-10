#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>

const char* ifname = "can0";

void send_frames(int sockfd) {
    struct can_frame frame;
    frame.can_id = 0x123;
    frame.can_dlc = 8;
    memcpy(frame.data, "\x11\x22\x33\x44\x55\x66\x77\x88", 8);

    while (true) {
        if (write(sockfd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write");
            break;
        }
        std::cout << "Sent CAN frame with ID 0x123" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1)); // Adjust as needed
    }
}

void receive_frames(int sockfd) {
    struct can_frame frame;
    while (true) {
        int nbytes = read(sockfd, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read");
            break;
        }
        if (nbytes < sizeof(struct can_frame)) {
            std::cerr << "Incomplete CAN frame" << std::endl;
            continue;
        }
        std::cout << "Received CAN frame with ID 0x" << std::hex << frame.can_id;
        std::cout << " Data:";
        for (int i = 0; i < frame.can_dlc; i++)
            std::cout << " 0x" << std::hex << +frame.data[i];
        std::cout << std::endl;
    }
}

int main() {
    int sockfd;
    struct sockaddr_can addr;
    struct ifreq ifr;

    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        perror("Socket");
        return 1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(sockfd, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }

    std::thread sender(send_frames, sockfd);
    std::thread receiver(receive_frames, sockfd);

    sender.join();
    receiver.join();

    close(sockfd);

    return 0;
}