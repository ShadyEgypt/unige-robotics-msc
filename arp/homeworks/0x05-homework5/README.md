# Watchdog Monitoring System in C

This program demonstrates a watchdog monitoring system implemented in C using processes and a named pipe (FIFO) for inter-process communication. It monitors three child processes for activity and generates an alert if any process becomes inactive for more than a specified timeout period.

## Program Structure

The program consists of three main components:

1. **Child Processes**: Simulate workload and periodically send a "ping" signal to the watchdog by writing their PID to a named pipe.
2. **Watchdog Process**: Reads the named pipe for signals from the child processes and monitors their activity. If a process remains inactive beyond a specified timeout period, the watchdog raises an alert.
3. **Parent Process**: Creates the named pipe, spawns the child and watchdog processes, and waits for their completion.

### Files Used
- `watchdog_pipe` (named pipe): A FIFO file located at `/tmp/watchdog_pipe`, used for communication between the watchdog and the child processes.

## Code Explanation

### Key Parts of the Code

1. **`init_processes` Function**:
   - Initializes the array `processes` with the PIDs and last activity times of each child process.

2. **`watchdog` Function**:
   - Continuously reads the pipe for signals (PIDs) from child processes.
   - Updates the last activity time for each process when it receives a ping.
   - Checks for inactive processes based on the defined `TIMEOUT` and prints an alert if a process exceeds this timeout.

3. **`child_process` Function**:
   - Each child process simulates work by sleeping for a random duration between 1 and 3 seconds.
   - After each work cycle, it writes its PID to the pipe as a ping signal.

4. **Main Function**:
   - Creates the named pipe (FIFO) for inter-process communication.
   - Forks three child processes and a watchdog process.
   - Initializes the `processes` array with the PIDs of the child processes.
   - Waits for all child and watchdog processes to complete.

### Timeout Configuration
The program defines a `TIMEOUT` of 5 seconds, meaning the watchdog will generate an alert if it doesn’t receive a ping from a process within 5 seconds.

## How to Compile and Run

1. **Compile the Program**:
   ```bash
   gcc -o solution1 solution-forks.c

2. **Output**
Process 8695 wrote to pipe
Process 8694 wrote to pipe
Process 8693 wrote to pipe
Watchdog: Received ping from process 8695
Watchdog: Received ping from process 8694
Process 8695 wrote to pipe
Process 8694 wrote to pipe
Process 8693 wrote to pipe
Watchdog: Received ping from process 8693
Watchdog: Received ping from process 8695
Process 8694 wrote to pipe
Process 8693 wrote to pipe
Process 8695 wrote to pipe
Watchdog: Received ping from process 8694
Watchdog: Received ping from process 8693
Watchdog: Received ping from process 8695
Process 8694 wrote to pipe
Process 8695 wrote to pipe
Process 8693 wrote to pipe
Watchdog: Received ping from process 8694
Watchdog: Received ping from process 8693