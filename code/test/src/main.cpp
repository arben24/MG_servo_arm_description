#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ESP32Servo.h>

IPAddress agent_ip(192, 168, 178, 54);
size_t agent_port = 8888;

char ssid[] = "SSID";
char psk[]= "WIFI_Password";

/*
micro-ROS-agent_ws sourcen
micro-Ros agent starten: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
um daten zu senden: ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, name: ['base', 'shoulder', 'elbow', 'wrist', 'gripper'], position: [75.0, 80.0, 40.0, 70.0, 35.0], velocity: [], effort: []}"
*/

rcl_subscription_t subscriber;
sensor_msgs__msg__JointState msg;
rclc_executor_t executor;

#define ARRAY_SIZE 6
#define ARRAY_LEN 200
#define JOINT_DOUBLE_LEN 20

const char *names[] = {"Umdrehung 21", "Umdrehung 24", "Umdrehung 27", "Umdrehung30", "Schieberegler 34", "Schieberegler 35"};
//const char *names[] = {"base", "shoulder", "elbow", "wrist", "gripper"};

Servo base, shoulder, elbow, wrist, gripper;
Servo* servos[5] = {&base, &shoulder, &elbow, &wrist, &gripper};

int actual_pos[5] = {75, 80, 40, 70, 35};
int start_array[5] = {70, 85, 45, 75, 30};
int joint_array[5];
int speed = 20;

void move_joints(int *joint_array, int move_speed) {
    int angle_array[5] = { *joint_array, *(joint_array + 1), *(joint_array + 2), *(joint_array + 3), *(joint_array + 4) };
    for (int s = 0; s < 5; s++) {
        if (angle_array[s] < actual_pos[s]) {
            for (int i = actual_pos[s]; i > angle_array[s]; i--) {
                servos[s]->write(i);
                delay(move_speed);
            }
        } else {
            for (int i = actual_pos[s]; i < angle_array[s]; i++) {
                servos[s]->write(i);
                delay(move_speed);
            }
        }
        actual_pos[s] = angle_array[s];
    }
}

void init_arm() {
    move_joints(start_array, speed);
}

void subscription_callback(const void *msgin) {
    Serial.print("callback");
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
    
    Serial.print("Received JointState: ");
    for (size_t i = 0; i < 5; i++) { //msg->position.size
        Serial.print(msg->position.data[i]);
        Serial.print(" ");
        joint_array[i] = msg->position.data[i];
    }
    Serial.println();

    //move_joints(joint_array, speed);
}

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial to initialize

    // Pin assignment
    base.attach(14);
    shoulder.attach(27);
    elbow.attach(26);
    wrist.attach(25);
    gripper.attach(33);
    delay(500);
    init_arm();
    delay(1000);

    // Initialisieren Sie die Micro-ROS Kommunikation
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    // Initialisieren Sie den Node
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        Serial.println("Error initializing support");
    }

    rcl_node_t node;
    rc = rclc_node_init_default(&node, "joint_state_subscriber_node", "", &support);
    if (rc != RCL_RET_OK) {
        Serial.println("Error initializing node");
    }

    // Initialisieren Sie den Subscriber
    rc = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states_test"
    );
    if (rc != RCL_RET_OK) {
        Serial.println("Error initializing subscription");
    }

    msg.name.data = (rosidl_runtime_c__String*)malloc(ARRAY_SIZE * sizeof(rosidl_runtime_c__String));
    if (msg.name.data == NULL) {
        Serial.println("Error allocating memory for msg.name.data");
    }
    msg.name.size = ARRAY_SIZE;
    msg.name.capacity = ARRAY_SIZE;

    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        msg.name.data[i].data = (char*)malloc((strlen(names[i]) + 1) * sizeof(char));
        if (msg.name.data[i].data == NULL) {
            Serial.println("Error allocating memory for name element");
        }
        strcpy(msg.name.data[i].data, names[i]);
        msg.name.data[i].size = strlen(names[i]);
        msg.name.data[i].capacity = strlen(names[i]) + 1;
    }

    msg.position.data = (double*)malloc(ARRAY_SIZE * sizeof(double));
    if (msg.position.data == NULL) {
        Serial.println("Error allocating memory for msg.position.data");
    }
    msg.position.size = ARRAY_SIZE;
    msg.position.capacity = ARRAY_SIZE;

    // Executor initialisieren und die Callback-Funktion verknüpfen
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (rc != RCL_RET_OK) {
        Serial.println("Error initializing executor");
    }
    rc = rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        Serial.println("Error adding subscription to executor");
    }

    Serial.println("Setup complete");
}

void loop() {
    // Spin den Executor, um Nachrichten zu verarbeiten
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(100);
}
