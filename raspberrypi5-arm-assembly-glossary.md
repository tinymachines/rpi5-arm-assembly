# ARM64 Assembly Programming Glossary for Raspberry Pi 5

## Table of Contents
1. [Architecture Terminology](#architecture-terminology)
2. [Assembly Programming Concepts](#assembly-programming-concepts)
3. [Memory Management Terms](#memory-management-terms)
4. [Instruction Reference](#instruction-reference)
5. [Register Architecture](#register-architecture)
6. [NEON/SIMD Concepts](#neonsimd-concepts)
7. [System Instructions](#system-instructions)
8. [Raspberry Pi 5 Hardware](#raspberry-pi-5-hardware)
9. [Development Tools](#development-tools)
10. [Performance & Optimization](#performance--optimization)

---

## Architecture Terminology

### AArch64
The 64-bit execution state of the ARMv8 architecture, also known as ARM64. Provides 31 general-purpose 64-bit registers (X0-X30), improved instruction set efficiency, and mandatory cryptographic extensions support.

### ARMv8.2-A
The specific ARM architecture version implemented in Raspberry Pi 5's Cortex-A76 cores. Includes features like half-precision floating-point, RAS (Reliability, Availability, Serviceability), and optional dot product instructions.

### Cortex-A76
The high-performance CPU core used in BCM2712, featuring a 13-stage pipeline, 4-wide decode, out-of-order execution, and significant performance improvements over previous generations. Operates up to 2.4GHz on Pi 5.

### Exception Levels (EL0-EL3)
- **EL0**: User mode - unprivileged execution for applications
- **EL1**: Kernel mode - OS kernel with system register access
- **EL2**: Hypervisor mode - virtualization support
- **EL3**: Secure monitor - highest privilege for security transitions

### PSTATE
Processor state containing condition flags (NZCV), exception level, and other control bits. Not a register but an abstraction of various processor state elements.

### Endianness
ARM64 supports both big-endian and little-endian modes, but Raspberry Pi 5 operates in little-endian mode where the least significant byte is stored at the lowest address.

---

## Assembly Programming Concepts

### Addressing Modes
Methods for specifying operand locations:
- **Immediate**: `[Xn, #imm]` - base + constant offset
- **Register**: `[Xn, Xm]` - base + register offset
- **Pre-indexed**: `[Xn, #imm]!` - update base before access
- **Post-indexed**: `[Xn], #imm` - update base after access

### AAPCS64
ARM 64-bit Application Binary Interface defining calling conventions, register usage, stack alignment (16-byte), and parameter passing rules. Critical for function interoperability.

### Condition Codes
4-bit values encoding test conditions based on NZCV flags:
- **EQ/NE**: Equal/Not equal (Z flag)
- **GT/LT/GE/LE**: Signed comparisons
- **HI/LO/HS/LS**: Unsigned comparisons
- **MI/PL**: Negative/Positive (N flag)

### Stack Frame
Memory structure for function local variables and saved registers. Frame pointer (X29) points to previous frame, link register (X30) stores return address.

### Literal Pool
Constants embedded in code sections, accessed via PC-relative addressing. Used for loading large immediate values that don't fit in instruction encoding.

### Section
ELF file segments like `.text` (code), `.data` (initialized data), `.bss` (uninitialized data), `.rodata` (read-only data).

---

## Memory Management Terms

### Memory Barriers
Instructions ensuring memory operation ordering:
- **DMB**: Data Memory Barrier - orders memory accesses
- **DSB**: Data Synchronization Barrier - ensures completion
- **ISB**: Instruction Synchronization Barrier - flushes pipeline

### Memory Types
- **Normal**: Cacheable memory for code/data, allows speculation
- **Device**: Non-cacheable for MMIO, no speculation, strict ordering
- **Normal-Tagged**: Supports Memory Tagging Extension (MTE)

### Shareability Domains
- **Non-shareable**: Private to processing element
- **Inner Shareable**: Within coherency domain (e.g., CPU cluster)
- **Outer Shareable**: System-wide sharing

### Translation Table
Multi-level page tables mapping virtual to physical addresses. ARM64 supports 4KB, 16KB, or 64KB page sizes with up to 4-level hierarchy.

### Cache Coherency
Hardware-maintained consistency between caches in multi-core systems. Pi 5's Cortex-A76 cores share coherent L3 cache.

### Atomicity
Operations that complete indivisibly. ARM64 provides exclusive monitors and atomic instructions (CAS, LDADD) for lock-free programming.

---

## Instruction Reference

### Data Movement Instructions

#### MOV - Move
```assembly
MOV Xd, Xn      ; Copy register
MOV Xd, #imm    ; Load immediate
```
**State Changes**: Xd = source value, no flags affected
**Notes**: Actually encoded as `ORR Xd, XZR, Xn`

#### LDR/STR - Load/Store Register
```assembly
LDR Xd, [Xn, #offset]   ; Load from memory
STR Xd, [Xn, #offset]   ; Store to memory
```
**State Changes**: LDR: Xd = memory[address]; STR: memory[address] = Xd
**Addressing**: Supports immediate, register, pre/post-indexed modes

#### LDP/STP - Load/Store Pair
```assembly
LDP X0, X1, [SP, #16]   ; Load two registers
STP X29, X30, [SP, #-16]! ; Store with pre-decrement
```
**State Changes**: Loads/stores two consecutive registers
**Performance**: More efficient than separate LDR/STR

### Arithmetic Instructions

#### ADD/SUB - Addition/Subtraction
```assembly
ADD Xd, Xn, Xm     ; Xd = Xn + Xm
ADDS Xd, Xn, #imm  ; Add and set flags
```
**State Changes**: 
- ADD: Xd = Xn + operand
- ADDS: Also sets NZCV flags

#### MUL/DIV - Multiply/Divide
```assembly
MUL Xd, Xn, Xm     ; Multiply
UDIV Xd, Xn, Xm    ; Unsigned divide
```
**State Changes**: 
- MUL: Xd = Xn × Xm (low 64 bits)
- UDIV: Xd = Xn ÷ Xm (truncated)

### Control Flow Instructions

#### B/BL - Branch
```assembly
B label            ; Branch to label
BL function        ; Branch with link (call)
```
**State Changes**: 
- B: PC = target
- BL: X30 = PC + 4, PC = target

#### B.cond - Conditional Branch
```assembly
CMP X0, X1
B.EQ equal_label   ; Branch if equal
```
**Conditions**: EQ, NE, GT, GE, LT, LE, HI, HS, LO, LS, MI, PL, VS, VC

#### RET - Return
```assembly
RET               ; Return to X30
RET Xn            ; Return to Xn
```
**State Changes**: PC = Xn (default X30)

### Logical Instructions

#### AND/ORR/EOR - Bitwise Operations
```assembly
AND Xd, Xn, Xm    ; Bitwise AND
ORR Xd, Xn, Xm    ; Bitwise OR
EOR Xd, Xn, Xm    ; Bitwise XOR
```
**State Changes**: Xd = Xn op Xm
**With S suffix**: Also updates NZCV flags

### Bit Manipulation

#### BFI/UBFX - Bitfield Operations
```assembly
BFI X0, X1, #8, #16   ; Insert bitfield
UBFX X0, X1, #8, #16  ; Extract unsigned
```
**State Changes**: Manipulates specific bit ranges

---

## Register Architecture

### General Purpose Registers

#### X0-X30 (64-bit) / W0-W30 (32-bit)
- **X0-X7**: Function arguments and return values
- **X8**: Indirect result register
- **X9-X15**: Caller-saved temporaries
- **X16-X17**: Intra-procedure call registers (IP0/IP1)
- **X18**: Platform register (may be reserved)
- **X19-X28**: Callee-saved registers
- **X29**: Frame pointer (FP)
- **X30**: Link register (LR)

**Important**: Writing to W register zeros upper 32 bits of X register

### Special Registers

#### SP - Stack Pointer
Must be 16-byte aligned. Modified by stack operations and function calls.

#### PC - Program Counter
Not directly accessible in AArch64. Modified by branch instructions.

#### XZR/WZR - Zero Register
Always reads as zero, writes are discarded. Register 31 in most encodings.

### System Registers

#### FPSR - Floating-Point Status Register
Contains cumulative exception flags:
- **IOC**: Invalid operation
- **DZC**: Division by zero
- **OFC**: Overflow
- **UFC**: Underflow
- **IXC**: Inexact

#### FPCR - Floating-Point Control Register
Controls rounding mode and exception handling

---

## NEON/SIMD Concepts

### Vector Registers

#### V0-V31
128-bit SIMD registers with multiple views:
- **Qn**: 128-bit quad-word
- **Dn**: 64-bit double-word (low half)
- **Sn**: 32-bit single-word
- **Hn**: 16-bit half-word

### Lane Specifications
- **V0.16B**: 16 lanes × 8-bit
- **V0.8H**: 8 lanes × 16-bit
- **V0.4S**: 4 lanes × 32-bit
- **V0.2D**: 2 lanes × 64-bit

### SIMD Instructions

#### LD1/ST1 - Vector Load/Store
```assembly
LD1 {V0.4S}, [X0]      ; Load 4 floats
LD1 {V0.S}[0], [X0]    ; Load to lane 0
```

#### FADD/FMUL - Vector Arithmetic
```assembly
FADD V0.4S, V1.4S, V2.4S  ; Add 4 floats
FMLA V0.4S, V1.4S, V2.4S  ; Multiply-accumulate
```

#### DUP - Duplicate Element
```assembly
DUP V0.4S, V1.S[0]     ; Broadcast lane to all
```

---

## System Instructions

### Exception Handling

#### SVC - Supervisor Call
```assembly
SVC #0                 ; System call to kernel
```
Generates synchronous exception to EL1

#### MSR/MRS - System Register Access
```assembly
MRS X0, MIDR_EL1      ; Read Main ID Register
MSR TTBR0_EL1, X0     ; Write Translation Table Base
```

### Cache Maintenance

#### DC - Data Cache Operations
- **DC IVAC**: Invalidate by address
- **DC CVAC**: Clean by address
- **DC CIVAC**: Clean and invalidate

#### IC - Instruction Cache Operations
- **IC IALLU**: Invalidate all
- **IC IVAU**: Invalidate by address

### Atomic Operations

#### LDAXR/STLXR - Exclusive Access
```assembly
retry:
    LDAXR W0, [X1]     ; Load exclusive with acquire
    ADD W0, W0, #1
    STLXR W2, W0, [X1] ; Store exclusive with release
    CBNZ W2, retry     ; Retry if failed
```

#### CAS - Compare and Swap (ARMv8.1+)
```assembly
CAS W0, W1, [X2]       ; Atomic compare and swap
```

---

## Raspberry Pi 5 Hardware

### BCM2712 SoC
Broadcom's latest SoC featuring:
- **CPU**: Quad-core Cortex-A76 @ 2.4GHz
- **Process**: 16nm FinFET
- **Cache**: 512KB L2 per core, 2MB shared L3
- **Memory**: 32-bit LPDDR4X-4267 interface

### RP1 Southbridge
Custom I/O controller connected via PCIe 2.0 x4:
- **GPIO**: 40-pin header control
- **USB**: 2×USB 3.0 + 2×USB 2.0
- **Interfaces**: UART, SPI, I2C, PWM, PIO
- **Base Address**: 0x1F00000000

### Memory Map
- **System RAM**: 0x1000000000+ (up to 16GB)
- **RP1 Peripherals**: 0x1F00000000
- **BCM2712 I/O**: 0x107C000000

### GIC-400
ARM Generic Interrupt Controller v2:
- **SGI**: Software Generated Interrupts (0-15)
- **PPI**: Private Peripheral Interrupts (16-31)
- **SPI**: Shared Peripheral Interrupts (32+)

### Performance Features
- **Pipeline**: 13-stage, 4-wide decode
- **Out-of-Order**: 128-instruction window
- **Branch Prediction**: 6K-entry BTB
- **Memory**: 17GB/s theoretical bandwidth

---

## Development Tools

### Assembler Directives

#### Section Control
```assembly
.text              ; Code section
.data              ; Initialized data
.bss               ; Uninitialized data
.section name      ; Custom section
```

#### Symbol Management
```assembly
.global _start     ; Export symbol
.equ CONSTANT, 42  ; Define constant
.align 4           ; Align to 16 bytes
```

#### Data Definition
```assembly
.byte 0x12         ; 8-bit value
.hword 0x1234      ; 16-bit value  
.word 0x12345678   ; 32-bit value
.quad 0x123456789AB ; 64-bit value
```

### Debug Support

#### CFI Directives
```assembly
.cfi_startproc     ; Start procedure frame info
.cfi_def_cfa_offset 16
.cfi_offset x29, -16
.cfi_endproc       ; End frame info
```

#### DWARF Information
Debug format for stack unwinding and source correlation

### Binary Utilities

#### objdump
```bash
objdump -d binary  ; Disassemble
objdump -S binary  ; With source
```

#### readelf
```bash
readelf -h binary  ; ELF header
readelf -S binary  ; Sections
```

---

## Performance & Optimization

### Pipeline Optimization
- **Instruction Scheduling**: Utilize 4-wide decode
- **Dependency Chains**: Minimize sequential dependencies
- **Branch Prediction**: Structure code for predictability

### Cache Optimization
- **Line Size**: 64 bytes
- **Alignment**: Align hot data to cache lines
- **Prefetching**: Use PLD instruction for known patterns

### NEON Optimization
- **Vectorization**: Process multiple data elements
- **Lane Operations**: Minimize cross-lane operations
- **Memory Access**: Use structured loads (LD2/3/4)

### Memory Ordering
- **Acquire/Release**: For synchronization primitives
- **Barriers**: DMB/DSB for explicit ordering
- **Atomics**: Prefer LSE atomics on ARMv8.1+

### Cortex-A76 Specifics
- **Execution Ports**: 8 independent queues
- **ASIMD Width**: 2×128-bit pipelines
- **Load/Store**: 2 AGUs, 68 in-flight loads
- **Latencies**: Integer multiply 2 cycles, FP 2 cycles

### Thermal Considerations
- **Active Cooling**: Recommended for sustained performance
- **Throttling**: Automatic at temperature thresholds
- **Monitoring**: Multiple thermal zones available

---

## Cross-References

- **AArch64** → See also: ARMv8, Exception Levels
- **Atomics** → See also: Memory Barriers, LDAXR/STLXR
- **BCM2712** → See also: Cortex-A76, RP1
- **Calling Convention** → See also: AAPCS64, Stack Frame
- **NEON** → See also: Vector Registers, SIMD Instructions
- **Memory Ordering** → See also: Barriers, Atomics

---

*This glossary provides a comprehensive reference for ARM64 assembly programming on Raspberry Pi 5, covering architecture fundamentals through advanced optimization techniques. For the latest updates and additional details, consult the ARM Architecture Reference Manual and Raspberry Pi documentation.*