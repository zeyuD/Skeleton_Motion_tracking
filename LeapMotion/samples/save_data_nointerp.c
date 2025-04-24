#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>

#include "LeapC.h"
#include "ExampleConnection.h"

#define STACK_SIZE     1 // number of frames to stack before sending
#define TARGET_FPS     90
#define PERIOD_NS      (1000000000ull / TARGET_FPS)   /* 11 111 111 ns */

int64_t lastFrameID = 0; // The last frame received

struct Data {
    float x, y, z, qx, qy, qz, qw, grab, roll, pitch, yaw;
    double timestamp;
};


// Function to convert quaternion to yaw, pitch, and roll
void quaternionToEuler(const LEAP_QUATERNION *q, float *roll, float *pitch, float *yaw) {
    // Roll (X-axis rotation)
    // *roll = atan2(2.0f * (q->x * q->y + q->w * q->z), q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z); // old
    *roll = atan2(2.0f * (q->z * q->y + q->w * q->x), q->w * q->w + q->z * q->z - q->y * q->y - q->x * q->x) * 180 / M_PI; // leap_origin

    // Pitch (Y-axis rotation)
    // float t2 = -2.0f * (q->x * q->z - q->w * q->y); // old
    float t2 = 2.0f * (q->w * q->y - q->x * q->z); // leap_origin
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

void write_data_to_csv(struct Data* data, size_t count, FILE* file) {
    for (size_t i = 0; i < count; ++i) {
        fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                data[i].timestamp, data[i].x, data[i].y, data[i].z,
                data[i].qx, data[i].qy, data[i].qz, data[i].qw,
                data[i].grab, data[i].roll, data[i].pitch, data[i].yaw);
    }
}

int main(int argc, char** argv) {
    OpenConnection();
    while (!IsConnected)
        millisleep(100); // wait a bit to let the connection complete

    printf("Connected.\n");
    LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
    if (deviceProps)
        printf("Using device %s.\n", deviceProps->serial);

    FILE* file = fopen("/home/zeyu/leapmotion/build/Release/leapc_example/_leap_.csv", "w");
    if (!file) {
        perror("fopen");
        return 1;
    }

    fprintf(file, "timestamp,x,y,z,qx,qy,qz,qw,grab,roll,pitch,yaw\n");

    struct Data data_stack[STACK_SIZE];
    size_t data_index = 0;
    struct Data data;
    float roll, pitch, yaw;

    // LEAP_TRACKING_EVENT* frame = GetFrame();
    // double start_time = (double)frame->info.timestamp / 1000000.0;
    double elapsed_time = 0;


    for (;;) {
        LEAP_TRACKING_EVENT* frame = GetFrame();
        if (frame && (frame->tracking_frame_id > lastFrameID)) {
            lastFrameID = frame->tracking_frame_id;
            for (uint32_t h = 0; h < frame->nHands; h++) {
                LEAP_HAND* hand = &frame->pHands[h];

                data.x = hand->palm.position.x;
                data.y = hand->palm.position.y;
                data.z = hand->palm.position.z;
                data.qx = hand->palm.orientation.x;
                data.qy = hand->palm.orientation.y;
                data.qz = hand->palm.orientation.z;
                data.qw = hand->palm.orientation.w;
                data.grab = hand->grab_strength;
                data.timestamp = (double)frame->info.timestamp / 1000000.0;
                // data.timestamp = get_unix_timestamp();

                // For robot control RPY
                quaternionToEuler_robot(&hand->palm.orientation, &roll, &pitch, &yaw);
                data.roll = roll;
                data.pitch = pitch;
                data.yaw = yaw;

                data_stack[data_index++] = data;

                if (data_index >= STACK_SIZE) {
                    write_data_to_csv(data_stack, data_index, file);
                    data_index = 0;
                }
            }
            // Early stop by a timer
            // elapsed_time = (double)frame->info.timestamp / 1000000.0 - start_time;
            // if (elapsed_time >= 8) {
            //     break;
            // }
        }
    }

    fclose(file);
    return 0;
}
