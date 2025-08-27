# Read-Write System with APB Protocol

This SystemVerilog project implements a modular, synthesizable **APB-based read/write communication system** that supports pipelined and buffered access to a memory-mapped slave interface. The design ensures that **high-throughput request handling** is achieved without stalling, by using a FIFO buffer and arbitration logic between read/write initiators.

The system simulates a common on-chip scenario where multiple subsystems (e.g., DMA engines or processors) issue read/write operations that must be **serialized, buffered, and forwarded** through a shared APB bus to a memory-mapped peripheral.

---

## 🔧 Design Specifications

1. **Write Has Higher Priority Than Read**  
   To maintain data consistency and avoid partial writes in systems with concurrent access, a **fixed-priority arbiter** gives precedence to write operations over reads.

2. **Request Buffering**  
   To handle bursts of activity without data loss or back-pressure, the system includes a **FIFO buffer that queues at least 16 requests** before initiating APB communication. This isolates high-speed requesters from the slower peripheral interface.

---

## 🧱 Key Modules

- **Fixed-Priority Arbiter**  
  Detects simultaneous read and write requests and issues a **one-hot grant** based on a fixed priority scheme. The write request is always granted first when both requests are active.

- **FIFO (Request Queue)**  
  A compact 2-bit wide, 16-depth FIFO used to encode and queue read/write requests from the arbiter. This prevents the APB master from being overwhelmed during request bursts and enables decoupling from the request source.

- **APB Master (APBM)**  
  Implements a standard 3-phase APB protocol: `IDLE → SETUP → ACCESS`. It pops operations from the FIFO and generates valid APB signals including address, write enable, and data. Read data is latched and reused as needed.

- **APB Slave (APBS)**  
  Responds to APB transactions by interfacing with an internal 16×32 memory. A **random delay** is introduced (via LFSR and counter) to model real-world memory/peripheral latency and test back-pressure behavior.
