# Message Queue-Based Communication System in C

This program demonstrates inter-process communication (IPC) using message queues in C. It simulates a system with a **server**, **sender**, and **two receivers**. The sender sends two integers to the server, which forwards the numbers to one of the receivers. The receivers calculate and print the mean and sum of the two numbers and send an acknowledgment back to the server. The server then forwards the acknowledgment to the sender. The system uses message queues to facilitate communication between the processes.

## Program Structure

The program is divided into the following main components:

1. **Server**: 
   - Manages the message queue and forwards messages between the sender and the receivers.
   - Processes the "quit" command to stop all child processes.
   
2. **Sender**: 
   - Sends two integers to the server and specifies which receiver should handle the request.
   - Waits for an acknowledgment from the server after sending the numbers.

3. **Receiver**:
   - Each receiver receives a message from the server, processes the two integers by calculating their mean and sum, and then sends an acknowledgment back to the server.

4. **Message Queue**:
   - A message queue is used to facilitate communication between the server, sender, and receivers. Each message contains the sender's data (numbers), the receiver's ID, and other control information.

### Files Used
- **Message Queue**: The message queue is created using the `msgget()` function with a unique key defined by `SERVER_QUEUE_KEY`.

## Code Explanation

### Key Parts of the Code

1. **`server()` Function**:
   - The server listens for messages from the sender (with message type `1`).
   - It checks if the message is a "quit" command (`q`). If so, it sends a quit command to the sender and both receivers and then terminates.
   - If the message is not a quit command, the server forwards the message to the appropriate receiver.

2. **`sender()` Function**:
   - The sender prompts the user to input two integers separated by a comma or type 'q' to quit.
   - If two integers are entered, the sender sends them to the server with the appropriate receiver ID.
   - After sending the numbers, the sender waits for an acknowledgment message from the server.

3. **`receiver()` Function**:
   - Each receiver listens for messages with a specific message type (1 for receiver 1, 2 for receiver 2).
   - When it receives the message with two integers, it calculates their mean and sum, then prints the results.
   - After processing, the receiver sends an acknowledgment back to the server.

4. **`main()` Function**:
   - The main function forks the server, sender, and receiver processes. It waits for all child processes to complete before exiting.

### Message Queue Setup

The message queue is identified using a unique key (`SERVER_QUEUE_KEY`). The program uses the `msgget()` function to create and access the queue. The structure of the message exchanged is:


1. **Compile the Program**:
   ```bash
   gcc -o solution solution.c

2. **Output**
Enter two integers separated by a comma, or 'q' to quit: 8, 6
[Sender] Sent 8 and 6 to receiver 1
[Receiver 1] Received 8 and 6. Mean: 7.000000, Sum: 14
[Sender] Acknowledgment received: ack

Enter two integers separated by a comma, or 'q' to quit: q
[Receiver 1] Received quit command, terminating.
[Receiver 2] Received quit command, terminating.