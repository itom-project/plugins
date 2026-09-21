# AGENTS.md — itom plugins

Guidance for AI coding agents working in the **itom plugins** repository (hardware and
algorithm plugins). Normally used as a submodule of `itomProject`.

## Plugin anatomy

Each plugin lives in its own folder and consists of:

- `CMakeLists.txt` — target definition, Qt moc/uic, install rules
- an **interface** class derived from `ito::AddInInterfaceBase`:
  metadata (`m_type`, `m_description`, `m_detaildescription`, `m_author`, `m_license`),
  mandatory/optional init parameters, `getAddInInst` / `closeThisInst`
- an **implementation** class derived from one of
  - `ito::AddInDataIO` — cameras, grabbers, serial/IO devices
  - `ito::AddInActuator` — motors, stages
  - `ito::AddInAlgo` — filters and algorithm widgets
- optionally `dialog*` / `dockWidget*` UI classes and a `docs/` folder

Reference implementations: `DummyGrabber`, `DummyMotor`, `BasicFilters`.
Skeletons: `itom/pluginTemplates` in the core repository.

## Hard rules

1. **Never change the core API from here** — only consume it. If something is missing in
   the core, say so instead of working around it.
2. **`ito::RetVal` for all error handling**; no exception may leave a plugin method.
3. **Semaphores:** every method that receives an `ItomSharedSemaphore* waitCond` must
   release it on *every* code path — use `ItomSharedSemaphoreLocker locker(waitCond);`
   and follow the pattern of the neighbouring plugins.
4. **Threading:** plugin instances run in their own thread; do not touch GUI objects
   directly, use signals (`parametersChanged`, `actuatorStatusChanged`,
   `targetChanged`, `newDataAvailable`).
5. **Parameters:** declare all parameters in the constructor in `m_params` with
   meta information and documentation strings; keys are lowerCamelCase and stable.
   `setParam` must validate via `apiValidateParam` and emit `parametersChanged`.
6. **DataObject outputs:** allocate/resize via `checkData()`, keep axis and tag meta
   data (`setAxisUnit`, `setAxisScale`, `setValueUnit`, `setTag`) up to date.
7. **Third party SDKs** are found via the CMake modules in `cmake/`; do not vendor
   binaries and do not hard-code SDK paths.

## Style

- `.clang-format` (`BasedOnStyle: Microsoft`, 4 spaces, `ColumnLimit: 100`,
  `PointerAlignment: Left`). Format only the lines you change.
- English identifiers and comments; user visible strings via `tr(...)`.
- Keep each plugin self-contained — no cross-plugin includes.

## Build

Plugins are built as part of `itomProject` with `BUILD_ITOM_PLUGINS=ON`.
Build only the plugin target you changed:

```powershell
cmake --build <build>/itomProject --config Debug --target <PluginName>
```

Never edit files inside the CMake binary directory.

## Quality gates

`pre-commit`: `check-yaml`, `end-of-file-fixer`, `trailing-whitespace`,
`fix-byte-order-marker`, `codespell`, `pyupgrade --py36-plus`, `sphinx-lint`.
Files end with a single newline, no trailing whitespace, no BOM.
