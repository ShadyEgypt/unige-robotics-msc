#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <unistd.h>
#include <sys/wait.h>

#define MSG_SIZE 128
#define SERVER_QUEUE_KEY 1234 // Unique key for message queue

struct message
{
    long msg_type;       // Message type (1 for sender to server, 2/3 for server to receivers)
    int n1, n2;          // Numbers to compute
    int receiver_id;     // Receiver ID (1 or 2)
    char text[MSG_SIZE]; // Text content, e.g., "q" for quit
};

void server()
{
    int msgid = msgget(SERVER_QUEUE_KEY, 0666 | IPC_CREAT);
    if (msgid == -1)
    {
        perror("msgget failed");
        exit(EXIT_FAILURE);
    }

    struct message msg;
    struct message ack_msg;

    while (1)
    {
        // Receive message from sender (msg_type = 1)
        if (msgrcv(msgid, &msg, sizeof(msg) - sizeof(long), 1, 0) == -1)
        {
            perror("msgrcv from sender failed");
            continue;
        }

        // Check for "quit" message to end receivers
        if (strcmp(msg.text, "q") == 0)
        {
            // Send quit command to both receivers and sender
            msg.msg_type = 2;
            if (msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1)
            {
                break;
            };
            msg.msg_type = 3;
            if (msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1)
            {
                break;
            };
            break;
        }

        // Forward message to the appropriate receiver
        msg.msg_type = msg.receiver_id; // Ensure correct receiver ID
        if (msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1)
        {
            perror("msgsnd to receiver failed");
        }
    }

    msgctl(msgid, IPC_RMID, NULL); // Remove message queue
    exit(EXIT_SUCCESS);
}

void sender()
{
    int msgid = msgget(SERVER_QUEUE_KEY, 0666 | IPC_CREAT);
    struct message msg;
    struct message ack_msg; // Structure to hold the acknowledgment
    char input_string[120];
    int toggle = 0;
    while (1)
    {
        printf("\nEnter two integers separated by a comma, or 'q' to quit: ");
        fflush(stdout);
        fgets(input_string, sizeof(input_string), stdin);

        if (input_string[0] == 'q')
        {
            msg.msg_type = 1; // Sending quit command to the server
            strcpy(msg.text, "q");
            if (msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1)
            {
                perror("msgsnd failed for quit command");
            }
            exit(EXIT_SUCCESS);
            break; // Exit the loop to terminate the sender
        }

        int n1, n2;
        if (sscanf(input_string, "%d,%d", &n1, &n2) == 2)
        {
            msg.msg_type = 1;             // Send to server
            msg.receiver_id = toggle + 2; // Determine receiver based on n1
            msg.n1 = n1;
            msg.n2 = n2;
            snprintf(msg.text, sizeof(msg.text), "%d,%d", n1, n2);
            if (msgsnd(msgid, &msg, sizeof(msg) - sizeof(long), 0) == -1)
            {
                perror("msgsnd failed to server");
            }
            printf("[Sender] Sent %d and %d to receiver %d\n", n1, n2, msg.receiver_id - 1);

            // Wait for acknowledgment from the server
            if (msgrcv(msgid, &ack_msg, sizeof(ack_msg) - sizeof(long), 1, 0) == -1)
            {
                perror("msgrcv failed while waiting for acknowledgment");
            }
            else
            {
                printf("[Sender] Acknowledgment received: %s\n", ack_msg.text);
            }
        }
        else
        {
            fprintf(stderr, "Invalid input. Please enter two integers separated by a comma.\n");
        }
        toggle = 1 - toggle;
    }

    printf("[Sender] Terminating sender process.\n");
    exit(EXIT_SUCCESS);
}

void receiver(int which)
{
    int msgid = msgget(SERVER_QUEUE_KEY, 0666);
    struct message msg;
    struct message ack_msg;
    int n1, n2;
    double mean;

    while (1)
    {
        // Receive message from server
        if (msgrcv(msgid, &msg, sizeof(msg) - sizeof(long), which + 1, 0) == -1)
        {
            perror("msgrcv from server failed");
            continue;
        }

        if (strcmp(msg.text, "q") == 0)
        {
            printf("[Receiver %d] Received quit command, terminating.\n", which);
            exit(EXIT_SUCCESS);
            break;
        }

        // Parse the received integers
        if (sscanf(msg.text, "%d,%d", &n1, &n2) == 2)
        {
            mean = (n1 + n2) / 2.0;
            printf("[Receiver %d] Received %d and %d. Mean: %f, Sum: %d\n", which, n1, n2, mean, n1 + n2);
            fflush(stdout);
        }
        else
        {
            fprintf(stderr, "[Receiver %d] Error parsing numbers.\n", which);
        }

        // Send acknowledgment to server
        ack_msg.msg_type = 1; // Acknowledgment message type
        strcpy(ack_msg.text, "ack");
        if (msgsnd(msgid, &ack_msg, sizeof(ack_msg) - sizeof(long), 0) == -1)
        {
            perror("msgsnd acknowledgment failed");
        }
    }
    exit(EXIT_SUCCESS);
}

int main()
{
    pid_t pid;

    // Start server process
    pid = fork();
    if (pid == 0)
    {
        server();
        exit(0);
    }

    // Start sender process
    pid = fork();
    if (pid == 0)
    {
        sender();
        exit(0);
    }

    // Start receiver 1
    pid = fork();
    if (pid == 0)
    {
        receiver(1);
        exit(0);
    }

    // Start receiver 2
    pid = fork();
    if (pid == 0)
    {
        receiver(2);
        exit(0);
    }

    // Wait for all child processes to finish
    for (int i = 0; i < 4; i++)
    {
        wait(NULL);
    }

    return 0;
}
