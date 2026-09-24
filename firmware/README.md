# RP2040 Firmware Documentation

<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="../docs/firmware/01-dc-motor-project.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="700" height="1">
  DC Motor Project
</div>
	
#

## Project Workflow
- The structure of the project is shown on the listing below. Basically we have two different type of package:
  - main &rarr; dc motor code
  - playground &rarr; experimental package to test the new feature before implemented to the main. 
  - 

  ``` bash
  .
  ├── .cargo
  │   └── config.toml
  ├── Cargo.toml  # Master Cargo for all packages
  ├── main                # main project            
  │   ├── build.rs          # Build Script
  │   ├── Cargo.toml        # Package Cargo for main
  │   ├── memory.x          # RP2040 memory layout
  │   └── src
  └── playground
      ├── flash_storage     # flash_storage project
      │  ├── build.rs         # Build Script
      │  ├── Cargo.toml       # Package Cargo for flash_storage
      │  ├── memory.x         # RP2040 memory layout
      │  └── src
      └── usb_communication   # usb_communication package (has the same project structure)
  ```

## Project Builder
## How to Create a New Project
#### Dependencies Setting Cargo.toml


#
<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="README.md"><img src="../assets/logo/home-button.png" alt="Home" height="30"></a>
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="350" height="1">
  <a href="../docs/firmware/01-dc-motor-project.md"><img src="../assets/logo/right-chevron.png" alt="Next >>" height="30"></a>
</div>
<div align="center">
  <img src="data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7" width="700" height="1">
  DC Motor Project
</div>
	
#