Antigravity AI Agent — Code Writing Workflow
C++ ROS2 & C STM32

Phase 1 — Understand Before Writing
Before generating any file, the agent must resolve:

What is the responsibility of this file? One file, one concern.
What already exists? Never duplicate logic. Check for existing nodes, drivers, or utilities before creating new ones.
What is the target? ROS2 node, hardware driver, HAL layer, message type, launch config, STM32 peripheral driver, etc.
What are the dependencies? Package names, message types, hardware peripherals, clock configs.

If any of these are unclear, ask once — concisely — before proceeding.

Phase 2 — File Generation Rules
General:

Generate only the files explicitly needed to fulfill the task.
Never generate: test files, example files, README stubs, placeholder files, or "future use" headers.
Never create a file and say "you can extend this later" — write it complete or don't write it.
File names must match the project's existing naming convention.

Comment Style — write comments like the developer who wrote the code, not like an AI explaining it:

No block comments describing what a function does if the function name already says it.
No // Step 1:, // Step 2: narration.
No // This function initializes the... style headers.
Use single-line comments only when the logic is non-obvious — hardware quirks, timing constraints, workarounds, register-level reasoning.
File-level comment: one line max, only if the module name doesn't speak for itself.
Never write a comment that mirrors the code directly below it.

✅ Good:
cpp// UART DMA must be re-armed after each transfer — hardware limitation
HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);
❌ Bad:
cpp// Initialize the UART with DMA mode to receive data into the buffer
HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);

Phase 3 — ROS2 C++ Rules
Node Structure:

Use composition (rclcpp::Node subclass) by default — not standalone executables unless required.
Constructor initializes everything: parameters, publishers, subscribers, timers.
Callbacks are private methods — no logic leaking into the constructor.
Spin logic goes in main.cpp — keep nodes decoupled from their executor.

Minimum files per node:
include/<pkg>/<node_name>.hpp
src/<node_name>.cpp
src/main.cpp        ← only if standalone executable
CMakeLists.txt      ← updated, not regenerated from scratch
package.xml         ← updated, not regenerated from scratch
Parameters:

Declare all with declare_parameter() in the constructor.
Never hardcode values that belong in a config.
Naming: snake_case, namespaced by function — e.g. sensor.update_rate_hz.

Publishers / Subscribers:

QoS must be explicit — never rely on silent defaults.
BEST_EFFORT for high-frequency sensor data, RELIABLE for control/commands.

CMakeLists.txt:

Only add what the node actually needs.
No find_package for unused dependencies.
No commented-out blocks or "optional" sections.


Phase 4 — STM32 C Rules
File structure:
Core/Inc/<peripheral>_driver.h
Core/Src/<peripheral>_driver.c
No abstraction layers unless the hardware actually requires it. No HAL wrappers that just re-call HAL.
Driver rules:

Init function sets up the peripheral completely — caller shouldn't need to do anything else before using it.
Public API: init, read, write, and IRQ handler if needed. Nothing else is exposed.
Static functions for internal helpers — nothing leaks into the header that isn't part of the API.
Error handling: return error codes, never silently ignore HAL return values.

Interrupts:

ISR bodies are short — set a flag or push to a buffer, nothing more.
Processing happens in the main loop or RTOS task, not in the ISR.

Memory:

No dynamic allocation (malloc/free) in drivers.
Buffers are statically declared and sized at compile time.

Registers / HAL:

If using direct register access, comment the register name and bit field — not what you're doing, but which hardware resource.


Phase 5 — Self-Evaluation Checklist (Run Before Every Output)
Functionality:

 Does the code compile with zero changes? Trace includes, types, function signatures mentally.
 Are all declared variables and functions actually used?
 Are edge cases handled or explicitly noted as out-of-scope?

Comments:

 Does any comment restate the code directly below it? → Delete it.
 Is any comment longer than one line without a real reason? → Trim it.
 Are there AI-style explanation headers? → Remove them.
 Would a senior engineer find any comment obvious or patronizing? → Delete it.

Files:

 Is every generated file necessary?
 Are there any test, example, or stub files in the output? → Remove them.
 Does any file exist just to "complete the structure"? → Remove it.

ROS2 specific:

 Are QoS profiles explicitly set?
 Are all parameters declared before use?
 Is CMakeLists.txt clean with no unused dependencies?

STM32 specific:

 Is the public API minimal and complete?
 Are all HAL return values checked?
 Is the ISR body short?

If any check fails → fix before outputting, not after.

Phase 6 — Output Format

Deliver files directly — no preamble like "Here is the code for your node:".
If multiple files are generated, list paths first, then content.
If modifying an existing file, show only the changed section with enough surrounding context to locate it — not the entire file unless it's small.
When the task is done, it's done. No "let me know if you want more features."


Hard Rules — Never Break

No test files unless explicitly requested.
No comments that describe self-explanatory code in plain English.
No placeholder files.
No unused includes, dependencies, or declarations.
No // TODO unless the task explicitly left something unresolved.
Never regenerate an entire existing file when only a section changes.
No verbose naming just to avoid writing comments — names should be clear, not sentences.