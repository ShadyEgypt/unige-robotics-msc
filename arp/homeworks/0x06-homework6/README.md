# Parent-Child FIFO Communication Program

This program demonstrates inter-process communication (IPC) in C using FIFOs (named pipes) with a parent process and two child processes. The goal is to have each child write characters at different rates to separate FIFOs, while the parent concurrently reads from the FIFOs and writes the data to an output file in real-time. The parent manages the reading process efficiently using the `select` system call, which enables it to handle multiple input sources without blocking.

## Code Overview

1. **FIFO Creation**:  
   The program begins by creating two FIFOs (named pipes), `fifo1` and `fifo2`, using the `mkfifo` function. These serve as the communication channels between the parent and the child processes.

2. **Child Processes**:  
   - Two child processes are created using `fork()`.
   - Each child writes a unique character ('A' or 'B') to its designated FIFO at a specific rate:
     - Child 1 writes 'A' to `fifo1` every 1 second.
     - Child 2 writes 'B' to `fifo2` every 0.5 seconds.
   - Each child runs for a maximum of 10 seconds before terminating.

3. **Parent Process**:  
   - The parent opens both FIFOs in non-blocking mode and starts a `select` loop to read from the FIFOs and write the characters sequentially to an output file named `output.txt`.
   - The `select` system call is used to monitor both FIFOs simultaneously.

### How `select` Works in This Code

The `select` function is the core of the parent process's reading mechanism. Here’s a breakdown of how it operates in this context:

- **File Descriptor Sets**:  
  - `select` uses file descriptor sets to track multiple input sources (in this case, `fifo1` and `fifo2`).
  - In each iteration of the parent’s loop, `select` waits for either FIFO to become ready for reading, preventing the program from blocking on just one FIFO at a time.

- **Non-blocking I/O**:  
  - Both FIFOs are opened in non-blocking mode, so the parent can check each FIFO's status without hanging.
  - If data is available on either FIFO, `select` returns immediately, and the parent reads from the FIFO and writes the character to the output file.
  - If no data is available, `select` waits for a timeout (1 second) before rechecking the status, allowing for an efficient polling mechanism.

- **Timeout Handling**:  
  - The 1-second timeout prevents the parent from waiting indefinitely and allows it to periodically check if both children have completed writing.

4. **Process Synchronization and Cleanup**:  
   - The parent process uses `waitpid` with the `WNOHANG` flag to check for child termination in a non-blocking way.
   - When both children have finished, the parent closes the FIFOs, waits for all child processes to complete, and then deletes the FIFO files.

## How to Run the Program

Compile and run the program with the following commands:

```bash
gcc -o solution1 solution1.c
./solution1



## output

sed s/A//g pic.txt | wc -c
20
sed s/B//g pic.txt | wc -c
10