from typing import List, Dict
import json
from itertools import groupby
import subprocess
from pathlib import Path  # 更安全的路径处理
import time

class VehicleInitializer:
    def __init__(self):
        self.vehicles: List[Dict] = []
    
    def add_vehicle(self, id: int, type: str, position: List[float], rotation: List[float]) -> None:
        self.vehicles.append({
            "id": id,
            "type": type,
            "position": position,
            "rotation": rotation
        })
    
    def save_to_json_vehicles(self, json_file: str) -> None:
        PRIORITY_ORDER = {
        "vehicle.imv.rb": 0,  # Highest priority (comes first)
        "vehicle.hrt.m10": 1,   # Lower priority (comes after)
        "other": 2
    }
    
        # Group vehicles by type and include their original ID for sorting
        grouped_data = {}
        for vehicle in self.vehicles:
            vehicle_type = vehicle["type"]
            if vehicle_type not in grouped_data:
                grouped_data[vehicle_type] = []
            
            detectRadius = 30000 if vehicle_type == "vehicle.hrt.m10" else 10000
            
            # Store the vehicle data along with its original ID
            grouped_data[vehicle_type].append({
                "id": vehicle["id"],  # Include the ID for secondary sorting
                "position": ",".join(map(str, vehicle["position"])),
                "rotation": ",".join(map(str, vehicle["rotation"])),
                "teamId": "1",
                "detectRadius": detectRadius
            })
        def sorting_key(item):
            # Primary sort by PRIORITY_ORDER, secondary sort by original ID
            vehicle_type, vehicles = item
            return (
                PRIORITY_ORDER.get(vehicle_type, float('inf')),  # Primary key: type priority
                min(vehicle["id"] for vehicle in vehicles)      # Secondary key: smallest ID in group
            )
        # First sort all vehicle types by priority and smallest ID
        sorted_groups = sorted(
            grouped_data.items(),
            key=sorting_key
        )
        # Then sort vehicles within each group by their ID
        for i, (vehicle_type, vehicles) in enumerate(sorted_groups):
            sorted_vehicles = sorted(vehicles, key=lambda v: v["id"])
            # Remove the temporary 'id' field if not needed in final output
            for vehicle in sorted_vehicles:
                vehicle.pop("id", None)  # Safely remove the id field if it exists
            sorted_groups[i] = (vehicle_type, sorted_vehicles)
        final_output = [
            {"type": vehicle_type, "vehicles": vehicles}
            for vehicle_type, vehicles in sorted_groups
        ]
        
        with open(json_file, "w") as f:
            json.dump(final_output, f, indent=4, ensure_ascii=False)
            
            
    # 启动外部初始化进程
    def execute_process(self, executable_path: str = "subject4/subject4") -> None: 
        try:
            executable_dir = str(Path(executable_path).parent)
            
            process = subprocess.Popen(
                ["gnome-terminal", "--", "./" + Path(executable_path).name],
                cwd=executable_dir,
            )
            
            print(f"Opened in new terminal: {executable_path}")
            print("Waiting 5s for initialization...")
            
            time.sleep(5)   
            
            print("Continuing with other tasks...")  
            
        except FileNotFoundError:
            print(f"Executable not found: {executable_path}")
