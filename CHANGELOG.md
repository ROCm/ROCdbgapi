# Changelog for AMD Debugger API (ROCdbgapi)

Full documentatino for AMD Debugger API is available at
[docs.amd.com](https://docs.amd.com/category/ROCDebugger%20API%20Guides).

## (Unreleased) rocm-dbgapi-0.70.0
### Changed
- The name reported for each agent is now based on the information stored
  in the [`pci.ids`](https://pci-ids.ucw.cz/) database.
- Return `AMD_DBGAPI_STATUS_ERROR_NOT_AVAILABLE` when querying
  `AMD_DBGAPI_REGISTER_INFO_DWARF` for a valid register which does not have
  an associated DWARF register number.

## (Unreleased) rocm-dbgapi-0.68.0
### Added
- Expose SGPRs mapped under `flat_scratch`/`xnack_mask`/`vcc`.
