# Copilot instructions — itom plugins

The tool-independent instructions for this repository live in
[`AGENTS.md`](../AGENTS.md). **Read and follow that file.**

Quick reminders:

- A plugin = interface class (`ito::AddInInterfaceBase`) + implementation
  (`ito::AddInDataIO`, `ito::AddInActuator` or `ito::AddInAlgo`) + own `CMakeLists.txt`.
- Use `DummyGrabber`, `DummyMotor` and `BasicFilters` as reference implementations.
- Return `ito::RetVal`; never let an exception escape a plugin method.
- Always release the `ItomSharedSemaphore*` (`ItomSharedSemaphoreLocker`) on every path.
- Declare parameters in `m_params` with meta info + doc string, validate in `setParam`
  and emit `parametersChanged(m_params)`.
- Do not modify the core API from here; do not include headers of other plugins.
- Formatting via `.clang-format`; format only the lines you touch.
