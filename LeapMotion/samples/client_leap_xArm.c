#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "LeapC.h"
#include "ExampleConnection.h"
#include <math.h>

#define SERVER_IP "10.9.149.27"
// #define SERVER_IP "127.0.0.1"
// #define SERVER_IP "10.9.157.137" // MacBook Pro WiFi IP
#define SERVER_PORT 8000
#define STACK_SIZE 1 // number of frames to stack before sending


// Function to convert quaternion to roll, pitch, and yaw
void quaternionToEuler(const LEAP_QUATERNION *q, float *roll, float *pitch, float *yaw) {
    // Roll (X-axis rotation)
    // *roll = atan2(2.0f * (q->x * q->y + q->w * q->z), q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z); // old
    *roll = atan2(2.0f * (q->z * q->y + q->w * q->x), q->w * q->w + q->z * q->z - q->y * q->y - q->x * q->x) * 180 / M_PI; // leap_origin

    // Pitch (Y-axis rotation)
    // double t2 = -2.0f * (q->x * q->z - q->w * q->y); // old
    double t2 = 2.0f * (q->w * q->y - q->x * q->z); // leap_origin
    if (t2 > 1.0f) {
        t2 = 1.0f;
    } else if (t2 < -1.0f) {
        t2 = -1.0f;
    }
    *pitch = asin(t2) * 180 / M_PI;

    // Yaw (Z-axis rotation)
    // *yaw = atan2(2.0f * (q->z * q->y + q->w * q->x), q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z); // old incorrect
    *yaw = atan2(2.0f * (q->x * q->y + q->w * q->z), q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z) * 180 / M_PI; // leap_origin
}

void quaternionToEuler_robot(const LEAP_QUATERNION *q, float *roll, float *pitch, float *yaw) {
    // Roll (X-axis rotation)
    *roll = atan2(2.0f * (q->y * (-q->x) + q->w * (-q->z)), q->w * q->w + q->y * q->y - q->x * q->x - q->z * q->z) * 180 / M_PI; // Robot

    // Pitch (Y-axis rotation)
    double t2 = 2.0f * (q->w * (-q->x) - (-q->z) * q->y); // Robot
    if (t2 > 1.0f) {
        t2 = 1.0f;
    } else if (t2 < -1.0f) {
        t2 = -1.0f;
    }
    *pitch = asin(t2) * 180 / M_PI;

    // Yaw (Z-axis rotation)
    *yaw = atan2(2.0f * ((-q->z) * (-q->x) + q->w * q->y), q->w * q->w + q->z * q->z - q->x * q->x - q->y * q->y) * 180 / M_PI; // Robot

}

struct Data {
    float x, y, z, roll, grab, pitch, yaw;
    double timestamp;
};

void serialize_data(struct Data* data, size_t count, char* buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size, "[");
    for (size_t i = 0; i < count; ++i) {
        char data_str[256];
        snprintf(data_str, sizeof(data_str),
                 "%f,%f,%f,%f,%f,%f,%f,%f",
                 data[i].x, data[i].y, data[i].z, data[i].roll, data[i].grab, data[i].timestamp, data[i].pitch, data[i].yaw);
                //  data[i].x, data[i].y, data[i].z, 0, data[i].grab, data[i].timestamp, 0, 0);

        printf("Derive hand command: %s\n", data_str);

        strncat(buffer, data_str, buffer_size - strlen(buffer) - 1);
        if (i < count - 1) {
            strncat(buffer, ",", buffer_size - strlen(buffer) - 1);
        }
    }
    strncat(buffer, "]", buffer_size - strlen(buffer) - 1);
}


int main(int argc, char** argv) {
    OpenConnection();
    while (!IsConnected)
        millisleep(100); // wait a bit to let the connection complete

    printf("Connected.\n");
    LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
    if (deviceProps)
        printf("Using device %s.\n", deviceProps->serial);


    int64_t lastFrameID = 0; // The last frame received
    float yaw, pitch, roll;
    struct Data data;
    struct sockaddr_in server_addr;
    struct Data data_stack[STACK_SIZE];
    size_t data_index = 0;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("inet_pton");
        return 1;
    }

    for (;;) {
        LEAP_TRACKING_EVENT* frame = GetFrame();
        if (frame && (frame->tracking_frame_id > lastFrameID)) {
            lastFrameID = frame->tracking_frame_id;
            for (uint32_t h = 0; h < frame->nHands; h++) {
                LEAP_HAND* hand = &frame->pHands[h];

                quaternionToEuler_robot(&hand->palm.orientation, &roll, &pitch, &yaw);

                data.x = hand->palm.position.x;
                data.y = hand->palm.position.y;
                data.z = hand->palm.position.z;
                data.roll = roll;
                data.grab = hand->grab_strength;
                data.pitch = pitch;
                data.yaw = yaw;
                data.timestamp = (double)frame->info.timestamp / 1000000.0;

                data_stack[data_index++] = data;

                if (data_index >= STACK_SIZE) {
                    char buffer[2048] = {0};
                    serialize_data(data_stack, data_index, buffer, sizeof(buffer));

                    // Print the data to the console
                    // printf("Data stack:\n");
                    // for (size_t i = 0; i < data_index; ++i) {
                    //     printf("- Hand position: %f, %f, %f\n", data_stack[i].x, data_stack[i].y, data_stack[i].z);
                    //     printf("  Hand orientation: %f, %f, %f\n", data_stack[i].roll, data_stack[i].pitch, data_stack[i].yaw);
                    //     printf("  Hand grab strength: %f\n", data_stack[i].grab);
                    //     printf("  Hand timestamp: %f\n", data_stack[i].timestamp);
                    // }
                    // printf("Data buffer: %s\n", buffer);

                    if (sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
                        perror("sendto");
                    }
                    data_index = 0;
                }
            }
        }
        // usleep(10000); // sleep to avoid excessive CPU usage
    }

    close(sock);
    return 0;
}
