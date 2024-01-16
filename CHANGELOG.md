# Changelog for AMD Debugger API (ROCdbgapi)

Full documentatino for AMD Debugger API is available at
[rocm.docs.amd.com](https://rocm.docs.amd.com/projects/ROCdbgapi/en/latest/index.html).

## rocm-dbgapi-0.74.0
### Added
- Added support to create and open core dumps.

## rocm-dbgapi-0.71.0
### Added
- Add support for gfx940, gfx941 and gfx942 architectures.

## rocm-dbgapi-0.70.0
### Changed
- The name reported for each agent is now based on the information stored
  in the [`pci.ids`](https://pci-ids.ucw.cz/) database.
- Return `AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE` when querying
  `AMD_DBGAPI_REGISTER_INFO_DWARF` for a valid register which does not have
  an associated DWARF register number.

### Known Issues
- Does not support debugging programs that use cooperative groups or CU masking
  for gfx1100, gfx1101, and gfx11102.  A restriction will be reported when
  attaching to a process that has already created cooperative group queues or
  CU masked queues.  Any attempt by the process to create a cooperative queue
  or CU masked queue when attached will fail.

- On gfx1100, gfx1101 and gfx1102, the library cannot debug a program past a
  `s_sendmsg sendmsg(MSG_DEALLOC_VGPRS)` instruction.  If an exception is
  delivered to a wave in an attached process after the wave has executed this
  instruction, the wave is killed and the exception is not reported in the
  debugger.

## rocm-dbgapi-0.68.0
### Added
- Expose SGPRs mapped under `flat_scratch`/`xnack_mask`/`vcc`.
