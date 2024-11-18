"""
JSON Files Combiner Script
-------------------------

This script combines multiple JSON files from a specified folder into a single consolidated JSON file.
It processes files with timestamps in their names and reformats the data structure.
Handles files containing multiple JSON objects per file.

Input:
------
1. Files naming format: YYYYMMDDHHMMSS_video_results.json
   Example: 20241029171127_video_results.json

2. Input JSON format (each line in file):
   {"action": "Sitting", "stage": {"dressing_stage": 1, "basic_stage": 5, "grooming_stage": 4, 
    "mouth_care_stage": 1, "transportation_stage": 3}, "duration": 5.66, "location": 3}
   {"action": "Grooming", "stage": {"dressing_stage": 1, "basic_stage": 5, "grooming_stage": 4, 
    "mouth_care_stage": 4, "transportation_stage": 3}, "duration": 0.0, "location": 3}

Output:
-------
Single JSON file with format:
{
    "data": [
        {
            "create_dt": "2024-10-28 07:00:00",  # Extracted from filename
            "action": "Sitting",
            "transportation_stage": "3",
            "grooming_stage": "4",
            "mouth_care_stage": "5",
            "basic_stage": "5",
            "dressing_stage": "4",
            "location": "1",
            "duration": "5.66"
        },
        ...
    ]
}

Key Features:
------------
1. Reads all JSON files from specified directory
2. Handles multiple JSON objects per file
3. Extracts datetime from filename
4. Flattens nested 'stage' structure
5. Converts all values to strings in output
6. Sorts entries by create_dt
7. Handles errors for individual files and JSON objects without stopping entire process
"""

import json
import os
from datetime import datetime

def combine_json_files(folder_path, output_file):
    """
    Combines multiple JSON files from a folder into a single JSON file with specified format.
    Handles files containing multiple JSON objects.
    
    Args:
        folder_path (str): Path to the folder containing JSON files
        output_file (str): Path for the output combined JSON file
    """
    combined_data = {"data": []}
    
    # Get all JSON files in the folder
    json_files = [f for f in os.listdir(folder_path) if f.endswith('.json')]
    
    for file_name in json_files:
        try:
            # Extract datetime from filename
            datetime_str = file_name[:14]  # Get first 14 characters
            create_dt = datetime.strptime(datetime_str, '%Y%m%d%H%M%S').strftime('%Y-%m-%d %H:%M:%S')
            
            # Read file line by line
            with open(os.path.join(folder_path, file_name), 'r') as file:
                for line_number, line in enumerate(file, 1):
                    try:
                        # Skip empty lines
                        if not line.strip():
                            continue
                            
                        # Parse JSON object from line
                        file_data = json.loads(line.strip())
                        
                        # Create new entry with desired format
                        new_entry = {
                            "create_dt": create_dt,
                            "action": file_data.get("action", ""),
                            "transportation_stage": str(file_data.get("stage", {}).get("transportation_stage", "")),
                            "grooming_stage": str(file_data.get("stage", {}).get("grooming_stage", "")),
                            "mouth_care_stage": str(file_data.get("stage", {}).get("mouth_care_stage", "")),
                            "basic_stage": str(file_data.get("stage", {}).get("basic_stage", "")),
                            "dressing_stage": str(file_data.get("stage", {}).get("dressing_stage", "")),
                            "location": str(file_data.get("location", "")),
                            "duration": str(file_data.get("duration", ""))
                        }
                        
                        combined_data["data"].append(new_entry)
                        
                    except json.JSONDecodeError as e:
                        print(f"Error parsing JSON in file {file_name}, line {line_number}: {str(e)}")
                        continue
                    except Exception as e:
                        print(f"Error processing line {line_number} in file {file_name}: {str(e)}")
                        continue
                
        except Exception as e:
            print(f"Error processing file {file_name}: {str(e)}")
            continue
    
    # Sort the data by create_dt
    combined_data["data"].sort(key=lambda x: x["create_dt"])
    
    # Write combined data to output file
    with open(output_file, 'w') as outfile:
        json.dump(combined_data, outfile, indent=2)
    
    print(f"Successfully combined data from {len(json_files)} files into {output_file}")
    print(f"Total number of entries: {len(combined_data['data'])}")

# Example usage
if __name__ == "__main__":
    # Replace these paths with your actual folder and output file paths
    folder_path = "test/test_data/dou_data"
    output_file = "test/test_data/combined_results.json"
    
    combine_json_files(folder_path, output_file)