# Elderly Activity Visualization System

## Overview
This ROS 2 package provides real-time visualization of elderly activities in a smart living room environment. Developed for demonstration at the National Center for Geriatrics and Gerontology (NCGG) in Nagoya, Japan, this system processes and visualizes various daily activities to help healthcare providers monitor and assess elderly residents' daily patterns.

## Demo Location
- **Institution**: National Center for Geriatrics and Gerontology (NCGG)
- **Location**: Nagoya, Japan
- **Environment**: Smart Living Room for Elderly Healthcare Monitoring

## Features
- Real-time data collection from smart living room sensors
- Processing of multiple activity categories:
  - Transportation activities
  - Grooming activities
  - Mouth care activities
  - Basic daily activities
  - Dressing activities
- Dynamic radar chart visualization
- Integration with Foxglove Studio for real-time monitoring
- Automatic duration calculation for each activity category

## Repository Structure
```
elderly_activity_visualization/
├── .github/
│   └── workflows/
│       └── ci.yaml
├── scripts/
│   ├── __init__.py
│   ├── data_fetcher.py          # Handles data fetching from NCGG server
│   ├── activity_processor.py     # Processes activity data
│   ├── visualization.py         # Creates radar charts
│   └── utils.py                 # Utility functions
├── config/
│   ├── params.yaml              # Configuration parameters
│   └── elderly_activity_foxglove_panel.json  # Foxglove layout configuration
├── launch/
│   └── visualization.launch.py  # Launch file
├── test/
│   ├── test_data_fetcher.py
│   ├── test_activity_processor.py
│   └── test_visualization.py
├── CMakeLists.txt
├── package.xml
├── requirements.txt
├── README.md
└── LICENSE
```

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8+
- Network access to NCGG server (192.168.1.109)
- Foxglove Studio (latest version)

## Installation

### 1. Set up ROS 2 Workspace
```bash
# Create a new workspace
mkdir -p ~/elderly_ws/src
cd ~/elderly_ws/src

# Clone the repository
git clone https://github.com/your-username/elderly_activity_visualization.git
cd ..
```

### 2. Install Dependencies
```bash
# Install ROS dependencies
sudo rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Install Python requirements
pip install -r requirements.txt
```

### 3. Build the Package
```bash
# Build
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### 1. Launch the Visualization System
```bash
ros2 launch elderly_activity_visualization visualization.launch.py
```

### 2. Configure Server Connection (if needed)
You can modify the server URL and fetch count using ROS 2 parameters:
```bash
ros2 launch elderly_activity_visualization visualization.launch.py server_url:=http://192.168.1.109/ncgg_icf_stage.php fetch_count:=100
```

### 3. Set Up Foxglove Visualization
1. Open Foxglove Studio
2. Connect to WebSocket (ws://localhost:9090)
3. Import the provided layout configuration:
   - Click on the "Layout" menu in the top navigation bar
   - Select "Import layout"
   - Navigate to `config/elderly_activity_foxglove_panel.json`
   - Click "Import"
4. The visualization panels will be automatically configured with:
   - Radar chart display for activity distribution
   - Raw data view for activity states
   - Timeline view for historical data
   - Custom parameter controls

### 4. View Visualizations
After importing the layout, you should see:
- Activity radar chart in the main panel
- Activity state table in the right panel
- Timeline view at the bottom
- Controls for adjusting visualization parameters

## Data Flow
1. Data Fetching
   - System connects to NCGG server
   - Fetches latest activity data every second
   - Filters out previously processed data

2. Data Processing
   - Categorizes activities into 5 main groups
   - Calculates duration for each activity
   - Tracks changes in activity states

3. Visualization
   - Generates radar charts showing activity distribution
   - Updates visualization in real-time
   - Publishes to ROS topics for Foxglove display

## ROS 2 Topics
- `/processed_activities`: Processed activity data
- `/activity_visualization`: Radar chart visualization
- `/rosbridge_websocket`: WebSocket connection for Foxglove

## Configuration
Default parameters can be modified in `config/params.yaml`:
```yaml
data_fetcher:
  ros__parameters:
    server_url: "http://192.168.1.109/ncgg_icf_stage.php"
    fetch_count: 100
    fetch_interval: 1.0
```

## Troubleshooting
1. Connection Issues
   ```bash
   # Check server connectivity
   ping 192.168.11.5
   
   # Verify ROS 2 topics
   ros2 topic list
   ros2 topic echo /processed_activities
   ```

2. Visualization Issues
   ```bash
   # Check if visualization node is running
   ros2 node list
   
   # Verify image publications
   ros2 topic echo /activity_visualization
   ```

3. Foxglove Layout Issues
   - If the layout import fails:
     1. Ensure you're using the latest version of Foxglove Studio
     2. Try manually creating a new layout and copying the settings
     3. Check if the layout file has proper read permissions

## Acknowledgments
- National Center for Geriatrics and Gerontology (NCGG), Nagoya
- Healthcare monitoring team at NCGG
- ROS 2 and Foxglove development communities