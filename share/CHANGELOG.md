# Change Log
All notable changes to HESAI lidar plugin will be documented in this file.
 
## [1.1.2] - 2022-10-27
 
### Added
- Get correction file from a local path if parser failed to parse it from the live sensor
 
### Changed
- Pass location of correction file to the plugin through the parameter `correction_path`
- Leave out duplicate parameter `session_id` in `run.sh`
- Leave out duplicate parameter `data_dir` when replay the bin file
- Move correction file `correction_at128` to new share folder

### Fixed

## [1.1.3] - 2023-2-10
 
- WARNING, this version is only compatible with DW5.8, for DRIVE ORIN machine
 
### Changed
- ReCompile the libraries based on DW5.8, drivework OS 6.0.5
- Remove unneccessary logs
- Remove unneccessary parameters in the scripts

### Fixed
