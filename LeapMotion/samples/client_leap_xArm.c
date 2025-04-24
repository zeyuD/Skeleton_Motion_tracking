#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <time.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>

#include "LeapC.h"
#include "ExampleConnection.h"

#define TARGET_FPS    100
#define PERIOD_US     (1000000 / TARGET_FPS)
#define STACK_SIZE     1
// #define SERVER_IP "10.9.167.67" // Legion WiFi IP
#define SERVER_IP "10.9.149.27" // XPS WiFi IP
// #define SERVER_IP "10.9.157.137" // MacBook Pro WiFi IP
#define SERVER_PORT 8000

struct Data {
    float x, y, z, grab, roll, pitch, yaw;
    double timestamp;
};

void quaternionToEuler_robot(const LEAP_QUATERNION *q, float *roll, float *pitch, float *yaw) {
    *roll = atan2(2.0f * (q->y * (-q->x) + q->w * (-q->z)), q->w * q->w + q->y * q->y - q->x * q->x - q->z * q->z) * 180 / M_PI;
    double t2 = 2.0f * (q->w * (-q->x) - (-q->z) * q->y);
    if (t2 > 1.0f) t2 = 1.0f;
    else if (t2 < -1.0f) t2 = -1.0f;
    *pitch = asin(t2) * 180 / M_PI;
    *yaw = atan2(2.0f * ((-q->z) * (-q->x) + q->w * q->y), q->w * q->w + q->z * q->z - q->x * q->x - q->y * q->y) * 180 / M_PI;
}

void write_data_to_csv(struct Data* data, size_t count, FILE* file) {
    for (size_t i = 0; i < count; ++i) {
        fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f\n",
                data[i].timestamp, data[i].x, data[i].y, data[i].z,
                data[i].grab, data[i].roll, data[i].pitch, data[i].yaw);
    }
}

void serialize_data(struct Data* data, size_t count, char* buffer, size_t buffer_size) {
    snprintf(buffer, buffer_size, "[");
    for (size_t i = 0; i < count; ++i) {
        char data_str[256];
        snprintf(data_str, sizeof(data_str),
                 "%f,%f,%f,%f,%f,%f,%f,%f",
                 data[i].x, data[i].y, data[i].z, data[i].roll, data[i].grab, data[i].timestamp, data[i].pitch, data[i].yaw);
                //  data[i].x, data[i].y, data[i].z, data[i].roll, 0, data[i].timestamp, data[i].pitch, data[i].yaw);
                //  data[i].x, data[i].y, data[i].z, 0, data[i].grab, data[i].timestamp, 0, 0);

        // printf("Derive hand command: %s\n", data_str);

        strncat(buffer, data_str, buffer_size - strlen(buffer) - 1);
        if (i < count - 1) {
            strncat(buffer, ",", buffer_size - strlen(buffer) - 1);
        }
    }
    strncat(buffer, "]", buffer_size - strlen(buffer) - 1);
}


int main(int argc, char** argv) {
    LEAP_CONNECTION* connHandle = OpenConnection();
    while (!IsConnected) millisleep(100);

    printf("Connected.\n");
    FILE* file = fopen("/home/zeyu/leapmotion/build/Release/leapc_example/interp_leap_.csv", "w");
    if (!file) {
        perror("fopen");
        return 1;
    }
    fprintf(file, "timestamp,x,y,z,grab,roll,pitch,yaw\n");

    struct Data data_stack[STACK_SIZE];
    struct sockaddr_in server_addr;
    size_t data_index = 0;
    struct Data data;

    LEAP_CLOCK_REBASER clockSynchronizer;
    LeapCreateClockRebaser(&clockSynchronizer);

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

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
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, NULL);

        clock_t cpuTime = (clock_t)(0.000001 * clock() / CLOCKS_PER_SEC);
        LeapUpdateRebase(clockSynchronizer, cpuTime, LeapGetNow());
        cpuTime = (clock_t)(0.000001 * clock() / CLOCKS_PER_SEC);

        int64_t targetFrameTime;
        LeapRebaseClock(clockSynchronizer, cpuTime, &targetFrameTime);

        uint64_t targetFrameSize = 0;
        eLeapRS result = LeapGetFrameSize(*connHandle, targetFrameTime, &targetFrameSize);

        if (result == eLeapRS_Success) {
            LEAP_TRACKING_EVENT* interpolatedFrame = malloc((size_t)targetFrameSize);
            result = LeapInterpolateFrame(*connHandle, targetFrameTime, interpolatedFrame, targetFrameSize);

            if (result == eLeapRS_Success && interpolatedFrame->tracking_frame_id != 0) {
                for (uint32_t h = 0; h < interpolatedFrame->nHands; ++h) {
                    LEAP_HAND* hand = &interpolatedFrame->pHands[h];

                    data.x = hand->palm.position.x;
                    data.y = hand->palm.position.y;
                    data.z = hand->palm.position.z;
                    data.grab = hand->grab_strength;
                    data.timestamp = (double)interpolatedFrame->info.timestamp / 1000000.0;

                    quaternionToEuler_robot(&hand->palm.orientation, &data.roll, &data.pitch, &data.yaw);

                    data_stack[data_index++] = data;
                    if (data_index >= STACK_SIZE) {
                        write_data_to_csv(data_stack, data_index, file);
                        char buffer[2048] = {0};
                        serialize_data(data_stack, data_index, buffer, sizeof(buffer));
                        if (sendto(sock, buffer, strlen(buffer), 0, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
                            perror("sendto");
                        }
                        data_index = 0;
                    }
                }
            }
            free(interpolatedFrame);
        }

        next_tick.tv_nsec += PERIOD_US * 1000;
        if (next_tick.tv_nsec >= 1000000000) {
            next_tick.tv_nsec -= 1000000000;
            next_tick.tv_sec  += 1;
        }
    }

    fclose(file);
    return 0;
}