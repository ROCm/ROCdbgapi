# AMD Debugger API Specification

## Introduction

The amd-dbgapi is a library that implements an AMD GPU debugger application
programming interface (API).  It provides the support necessary for a client
of the library to control the execution and inspect the state of supported
commercially available AMD GPU devices.

The term client is used to refer to the application that uses this API.

The term library is used to refer to the implementation of this interface
being used by the client.

The term <em>AMD GPU</em> is used to refer to commercially available AMD GPU
devices supported by the library.

The term inferior is used to refer to the process being debugged.

The library does not provide any operations to perform symbolic mappings,
code object decoding, or stack unwinding.  The client must use the AMD GPU
code object ELF ABI defined in [User Guide for AMDGPU Backend - Code Object](https://llvm.org/docs/AMDGPUUsage.html#code-object), together with the AMD
GPU debug information DWARF and call frame information CFI ABI define in
[User Guide for AMDGPU Backend - Code Object - DWARF](https://llvm.org/docs/AMDGPUUsage.html#dwarf) to perform those tasks.

The library does not provide operations for inserting or managing
breakpoints.  The client must write the architecture specific breakpoint
instruction provided by the
AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION query into the loaded
code object memory to set breakpoints.  For resuming from breakpoints the
client must use the displaced stepping mechanism provided by
amd_dbgapi_displaced_stepping_start and
amd_dbgapi_displaced_stepping_complete in conjunction with the
amd_dbgapi_wave_resume in single step mode.  In order to determine the
location of stopped waves the client must read the architecture specific
program counter register available using the
AMD_DBGAPI_ARCHITECTURE_INFO_PC_REGISTER query and adjust it by the amount
specified by the
AMD_DBGAPI_ARCHITECTURE_INFO_BREAKPOINT_INSTRUCTION_PC_ADJUST query.

The client is responsible for checking that only a single thread at a time
invokes a function provided by the library.  A callback (see \ref
callbacks_group) invoked by the library must not itself invoke any function
provided by the library.

The library implementation uses the native operating system to inspect and
control the inferior.  Therefore, the library must be executed on the same
machine as the inferior.

A library instance is defined as the period between a call to
amd_dbgapi_initialize and a matching call to amd_dbgapi_finalize.

The library uses opaque handles to refer to the entities that it manages.  A
handle value should not be modified directly.  See the handle definitions
for information on the lifetime and scope of handles of that type.  Handles
are invalidated outside their lifetime, scope, or single library instance.
If a handle is returned by an operation in one library instance which then
becomes invalidated, then any operation using that handle in the same
library instance will return an invalid handle error code.  However, it is
undefined to use a handle created by an operation in one library instance in
the operations of another library instance.  A handle value is globally
unique within each library instance.  This is true even if the handle
becomes invalidated: handle values are not reused within a library instance.
Every handle with handle value of 0 is reserved to indicate the handle
does not reference an entity.

When the library is first loaded it is in the uninitialized state with the
logging level set to AMD_DBGAPI_LOG_LEVEL_NONE.

## AMD GPU Execution Model

In this section the AMD GPU execution model is described to provide
background to the reader if they are not familiar with this environment.
The AMD GPU execution model is more complicated than that of a traditional
CPU because of how GPU hardware is used to accelerate and schedule the very
large number of threads of execution that are created on GPUs.

Chapter 2 of the [HSA Programmer's Reference Manual][hsa-prm] provides an
introduction to this execution model.  Note that the AMD ROCm compilers
compile directly to ISA and do not use the HSAIL intermediate language.
However, the ROCr low-level runtime and ROCgdb debugger use the same
terminology.

In this model, a CPU process may interact with multiple AMD GPU devices,
which are termed agents.  A Process Address Space Identifier (PASID) is
created for each process that interacts with agents.  An agent can be
executing code for multiple processes at once.  This is achieved by mapping
the PASID to one of a limited set of Virtual Memory Identifiers (VMIDs).
Each VMID is associated with its own page table.

The AMD GPU device driver for Linux, termed the Kernel Mode Driver (KMD),
manages the page tables used by each GPU so they correlate with the CPU page
table for the corresponding process.  The CPU and GPU page tables do not
necessarily map all the same memory pages but pages they do have in common
have the same virtual address.  Therefore, the CPU and GPUs have a unified
address space.

Each GPU includes one or more Microcode Engines (ME) that can execute
microcode firmware.  This firmware includes a Hardware Scheduler (HWS) that,
in collaboration with the KMD, manages which processes, identified by a
PASID, are mapped onto the GPU using one of the limited VMIDs.  This mapping
configures the VMID to use the GPU page table that corresponds to the PASID.
In this way, the code executing on the GPU from different processes is
isolated.

Multiple software submission queues may be created for each agent.  The GPU
hardware has a limited number of pipes, each of which has a fixed number of
hardware queues.  The HWS, in collaboration with the KMD, is responsible for
mapping software queues onto hardware queues.  This is done by multiplexing
the software queues onto hardware queues using time slicing.  The software
queues provide a virtualized abstraction, allowing for more queues than are
directly supported by the hardware.  Each ME manages its own set of pipes
and their associated hardware queues.

To execute code on the GPU, a packet must be created and placed in a
software queue.  This is achieved using regular user space atomic memory
operations.  No Linux kernel call is required.  For this reason, the queues
are termed user mode queues.

The AMD ROCm platform uses the Asynchronous Queuing Language (AQL) packet
format defined in the [HSA Platform System Architecture
Specification][hsa-sysarch].  Packets can request GPU management actions
(for example, manage memory coherence) and the execution of kernel
functions.  The ME firmware includes the Command Processor (CP) which,
together with fixed-function hardware support, is responsible for detecting
when packets are added to software queues that are mapped to hardware
queues.  Once detected, CP is responsible for initiating actions requested
by the packet, using the appropriate VMID when performing all memory
operations.

Dispatch packets are used to request the execution of a kernel function.
Each dispatch packet specifies the address of a kernel descriptor, the
address of the kernel argument block holding the arguments to the kernel
function, and the number of threads of execution to create to execute the
kernel function.  The kernel descriptor describes how the CP must configure
the hardware to execute the kernel function and the starting address of the
kernel function code.  The compiler generates a kernel descriptor in the
code object for each kernel function and determines the kernel argument
block layout.  The number of threads of execution is specified as a grid,
such that each thread of execution can identify its position in the grid.
Conceptually, each of these threads executes the same kernel code, with the
same arguments.

The dispatch grid is organized as a three-dimensional collection of
workgroups, where each workgroup is the same size (except for potential
boundary partial workgroups).  The workgroups form a three-dimensional
collection of work-items.  The work-items are the threads of execution.  The
position of a work-item is its zero-based three-dimensional position in a
workgroup, termed its work-item ID, plus its workgroup's three-dimensional
position in the dispatch grid, termed its workgroup ID.  These
three-dimensional IDs can also be expressed as a zero-based one-dimensional
ID, termed a flat ID, by simply numbering the elements in a natural manner
akin to linearizing a multi-dimensional array.

Consecutive work-items, in flat work-item ID order, of a workgroup are
organized into fixed size wavefronts, or waves for short.  Each work-item
position in the wave is termed a lane, and has a zero-base lane ID.  The
hardware imposes an upper limit on the number of work-items in a workgroup
but does not limit the number of workgroups in a dispatch grid.  The
hardware executes instructions for waves independently.  But the lanes of a
wave all execute the same instruction jointly.  This is termed Single
Instruction Multiple Thread (SIMT) execution.

Each hardware wave has a set of registers that are shared by all lanes of
the wave, termed scalar registers.  There is only one set of scalar
registers for the whole wave.  Instructions that act on the whole wave,
which typically use scalar registers, are termed scalar instructions.

Additionally, each wave also has a set of vector registers that are
replicated so each lane has its own copy.  A set of vector registers can be
viewed as a vector with each element of the vector belonging to the
corresponding lane of the wave.  Instructions that act on vector registers,
which produce independent results for each lane, are termed vector
instructions.

Each hardware wave has an execution mask that controls if the execution of a
vector instruction should change the state of a particular lane.  If the
lane is masked off, no changes are made for that lane and the instruction is
effectively ignored.  The compiler generates code to update the execution
mask which emulates independent work-item execution.  However, the lanes of
a wave do not execute instructions independently.  If two subsets of lanes
in a wave need to execute different code, the compiler will generate code to
set the execution mask to execute the subset of lanes for one path, then
generate instructions for that path.  The compiler will then generate code
to change the execution mask to enable the other subset of lanes, then
generate code for those lanes.  If both subsets of lanes execute the same
code, the compiler will generate code to set the execution mask to include
both subsets of lanes, then generate code as usual.  When only a subset of
lanes is enabled, they are said to be executing divergent control flow.
When all lanes are enabled, they are said to be executing wave uniform
control flow.

Not all MEs have the hardware to execute kernel functions.  One such ME is
used to execute the HWS microcode and to execute microcode that manages a
service queue that is used to update GPU state.  If the ME does support
kernel function execution it uses fixed-function hardware to initiate the
creation of waves.  This is accomplished by sending requests to create
workgroups to one or more Compute Units (CUs).  Requests are sent to create
all the workgroups of a dispatch grid.  Each CU has resources to hold a
fixed number of waves and has fixed-function hardware to schedule execution
of these waves.  The scheduler may execute multiple waves concurrently and
will hide latency by switching between the waves that are ready to execute.
At any point of time, a subset of the waves belonging to workgroups in a
dispatch may be actively executing.  As waves complete, the waves of
subsequent workgroup requests are created.

Each CU has a fixed amount of memory from which it allocates vector and
scalar registers.  The kernel descriptor specifies how many registers to
allocate for a wave.  There is a tradeoff between how many waves can be
created on a CU and the number of registers each can use.

The CU also has a fixed size Local Data Store (LDS).  A dispatch packet
specifies how much LDS each workgroup is allocated.  All waves in a
workgroup are created on the same CU.  This allows the LDS to be used to
share data between the waves of the same workgroup.  There is a tradeoff
between how much LDS a workgroup can allocate, and the number of workgroups
that can fit on a CU.  The address of a location in a workgroup LDS
allocation is zero-based and is a different address space than the global
virtual memory.  There are specific instructions that take an LDS address to
access it.  There are also flat address instructions that map the LDS
address range into an unused fixed aperture range of the global virtual
address range.  An LDS address can be converted to or from a flat address by
offsetting by the base of the aperture.  Note that a flat address in the LDS
aperture only accesses the LDS workgroup allocation for the wave that uses
it.  The same address will access different LDS allocations if used by waves
in different workgroups.

The dispatch packet specifies the amount of scratch memory that must be
allocated for a work-item.  This is used for work-item private memory.
Fixed-function hardware in the CU manages per wave allocation of scratch
memory from pre-allocated global virtual memory mapped to GPU device memory.
Like an LDS address, a scratch address is zero-based, but is per work-item
instead of per workgroup.  It maps to an aperture in a flat address.  The
hardware swizzles this address so that adjacent lanes access adjacent DWORDs
(4 bytes) in global memory for better cache performance.

For an AMD Radeon Instinct™ MI60 GPU the workgroup size limit is 1,024
work-items, the wave size is 64, and the CU count is 64.  A CU can hold up
to 40 waves (this is limited to 32 if using scratch memory).  Therefore, a
workgroup can comprise between 1 and 16 waves inclusive, and there can be up
to 2,560 waves, making a maximum of 163,840 work-items.  A CU is organized
as 4 Execution Units (EUs) also referred to as Single Instruction Multiple
Data units (SIMDs) that can each hold 10 waves.  Each SIMD has 256 64-wide
DWORD vector registers and each CU has 800 DWORD scalar registers.  A single
wave can access up to 256 64-wide vector registers and 112 scalar registers.
A CU has 64KiB of LDS.

## Supported AMD GPU Architectures

The following AMD GPU architectures are supported:

- gfx900 (AMD Vega 10)
- gfx906 (AMD Vega 7nm also referred to as AMD Vega 20)
- gfx908 (AMD Instinct™ MI100 accelerator)
- gfx90a (Aldebaran)
- gfx1010 (Navi10)
- gfx1011 (Navi12)
- gfx1012 (Navi14)
- gfx1030 (Sienna Cichlid)
- gfx1031 (Navy Flounder)
- gfx1032 (Dimgrey Cavefish)

For more information about the AMD ROCm ecosystem, please refer to:

- https://docs.amd.com/

## Known Limitations and Restrictions

The AMD Debugger API library implementation currently has the following
restrictions.  Future releases aim to address these restrictions.

1. The following _get_info queries are not yet implemented:

- AMD_DBGAPI_QUEUE_INFO_ERROR_REASON
- AMD_DBGAPI_QUEUE_INFO_STATE

2. On a AMD_DBGAPI_STATUS_FATAL error the library does fully reset the
internal state and so subsequent functions may not operate correctly.

3. amd_dbgapi_process_next_pending_event returns
AMD_DBGAPI_EVENT_KIND_WAVE_STOP events only for AQL queues.  PM4 queues
that launch wavefronts are not supported.

4. amd_dbgapi_queue_packet_list returns packets only for AQL queues.

5. Generation of the AMD_DBGAPI_EVENT_KIND_QUEUE_ERROR event, the
AMD_DBGAPI_EVENT_INFO_QUEUE query, and the generation of
AMD_DBGAPI_EVENT_KIND_WAVE_COMMAND_TERMINATED events for waves with
pending single step requests when a queue enters the queue error state,
have not been implemented.

6. By default, for some architectures, the AMD GPU device driver for Linux
causes all wavefronts created when the library is not attached to the
process by amd_dbgapi_process_attach to be unable to query the
wavesfront's AMD_DBGAPI_WAVE_INFO_DISPATCH,
AMD_DBGAPI_WAVE_INFO_WORKGROUP_COORD, or
AMD_DBGAPI_WAVE_INFO_WAVE_NUMBER_IN_WORKGROUP, or workgroup's
AMD_DBGAPI_WORKGROUP_INFO_DISPATCH, or
AMD_DBGAPI_WORKGROUP_INFO_WORKGROUP_COORD.  This does not affect
wavefronts and workgroups created while the library is attached to the
process which are always capable of reporting this information.

If the HSA_ENABLE_DEBUG environment variable is set to "1" when the
inferior's runtime is successfully enabled (see
AMD_DBGAPI_EVENT_KIND_RUNTIME), then this information will be available
for all architecture even for wavefronts created when the library was not
attached to the process.  Setting this environment variable may very
marginally reduce wavefront launch latency for some architectures for
very short lived wavefronts.

amd_dbgapi_wave_get_info

7. Reads and writes to the region address space is not supported and will
always report AMD_DBGAPI_STATUS_ERROR_MEMORY_ACCESS.

8. The AMD_DBGAPI_WAVE_STOP_REASON_FP_and
AMD_DBGAPI_WAVE_STOP_REASON_INT-stop reasons (see
amd_dbgapi_wave_stop_reasons_t) are not reported for enabled arithmetic
exceptions if the DX10_CLAMP bit in the MODE register is set.  This
happens if the DX10_CLAMP kernel descriptor field is set.

9. The library does not support single root I/O virtualization (SR-IOV) on
any AMD GPU architecture that supports it.  That includes gfx1030,
gfx1031, and gfx1032.

## References

1. Advanced Micro Devices: [www.amd.com](https://www.amd.com/)
2. AMD ROCm Ecosystem: [docs.amd.com](https://docs.amd.com/)
3. Bus:Device.Function (BDF) Notation:
[wiki.xen.org/wiki/Bus:Device.Function_(BDF)_Notation](https://wiki.xen.org/wiki/Bus:Device.Function_(BDF)_Notation)
4. HSA Platform System Architecture Specification:
[www.hsafoundation.com/html_spec111/HSA_Library.htm#SysArch/Topics/SysArch_title_page.htm](http://www.hsafoundation.com/html_spec111/HSA_Library.htm#SysArch/Topics/SysArch_title_page.htm)
5. HSA Programmer's Reference Manual:
[www.hsafoundation.com/html_spec111/HSA_Library.htm#PRM/Topics/PRM_title_page.htm](http://www.hsafoundation.com/html_spec111/HSA_Library.htm#PRM/Topics/PRM_title_page.htm)
6. Semantic Versioning: [semver.org](https://semver.org)
7. The LLVM Compiler Infrastructure: [llvm.org](https://llvm.org/)
8. User Guide for AMDGPU LLVM Backend: [llvm.org/docs/AMDGPUUsage.html](https://llvm.org/docs/AMDGPUUsage.html)

[amd]:
https://www.amd.com/
"Advanced Micro Devices"

[bdf]:
https://wiki.xen.org/wiki/Bus:Device.Function_(BDF)_Notation
"[Bus:Device.Function (BDF) Notation]"

[hsa-prm]:
http://www.hsafoundation.com/html_spec111/HSA_Library.htm#PRM/Topics/PRM_title_page.htm
"HSA Programmer's Reference Manual"

[hsa-sysarch]:
http://www.hsafoundation.com/html_spec111/HSA_Library.htm#SysArch/Topics/SysArch_title_page.htm
"HSA Platform System Architecture Specification"

[llvm]:
https://llvm.org/
"The LLVM Compiler Infrastructure"

[llvm-amdgpu]:
https://llvm.org/docs/AMDGPUUsage.html
"User Guide for AMDGPU LLVM Backend"

[rocm]:
https://rocmdocs.amd.com/
"AMD ROCm Ecosystem"

[semver]:
https://semver.org/
"Semantic Versioning"
