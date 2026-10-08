# System Identification Script

import questionary
from pathlib import Path

import Config.Motor0
import Tool.SystemIdentification
from Tool.visualize import printg, printy

# -------------------------------------- Initialization -------------------------------------- #
asset_dir = Path(__file__).resolve().parent.parent / "assets/01_System_Identification/open-loop-responses"

motor = Tool.SystemIdentification.MotorOptimization(Config.Motor0, asset_dir=asset_dir)
printy("DC Motor System Identification")

option = questionary.select(
    "Please select the System Identification Method\n",
    choices=["differential_evolution", "least_square"],
    qmark="",
    instruction="(Use arrow keys and Enter to select; Ctrl+C to cancel)",
).ask()

if option is not None:
    printg(f"Selected Method: {option}")
    motor.run_system_identification(method=option)
else:
    printy("Selection Cancelled.")
    
# -------------------------------------- Playground -------------------------------------- #
