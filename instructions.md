# INSTRUCTIONS

This repository contains the steps to:

1. Collect expert data using the KUKA arm.
2. Process log files to convert them into readable CSV formats.
3. Replay expert trajectories.
4. Convert existing data into PKL files specifically for SERL.

## Initial Setup
First, build this package and source it:
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 1. Collecting Expert Data Using the KUKA Arm
- Ensure that the Arduino board and switches are connected to the robot.
- Start the **HandGuiding** application on the robot using the pendant. This application initiates the data collection process but does not enable hand guidance yet.
- To enable hand guidance, run the script:
  ```bash
  data_collector/data_collector/arduino-python.py
  ```
- When the script is executed, you will be asked to enter either **X** or **Y**:
  - **Y**: Enables hand guidance.
  - **X**: Disables hand guidance **AND** stops the data collection.

### Recording a Trajectory
1. Press **Y** to enable hand guidance.
2. Execute the expert data collection for ONE trajectory.
3. Press **X** to stop the data collection and handguidance for that trajectory.

Note that we will need to do this separately for each trajectory.

Repeat this process for as many trajectories as needed. The collected data will be stored in log format on the pendant. You can access these log files from your Windows system. Note that there will be one log file generated for each trajectory.

---

## 2. Converting Log Files to Readable CSV Format
1. Copy the collected log files from the pendant to a folder on your machine.
2. Update the values in `config.json`:
   - **log_folder_path**: Path to the folder where your `.log` files are stored.
   - **processed_csv_folder_path**: Path to the folder where you want to save the processed CSV files.
3. Run the following script to process the logs:
   ```bash
   data_collector/data_collector/parse_robot_log.py
   ```

---

## 3. Replaying Expert Trajectories
### Overview
We can replay only one trajectory at a time. This process involves starting the cameras and the LBRServer, recording images and joint positions, and converting joint positions into Cartesian positions.

### Steps
1. Launch the LBRServer on the pendant and execute the following commands:
   ```bash
   ros2 launch lbr_bringup bringup.launch.py model:=iiwa14 sim:=false rviz:=true moveit:=true
   ros2 launch realsense2_camera rs_multi_camera_launch_sync.py publish_tf:=false serial_no1:="'840412060409'" serial_no2:="'932122060300'"
   ```
2. Update `config.json`:
   - Replace the **replay_csv_name** variable with the name of the CSV file containing the trajectory we want to replay (include the `.csv` extension).

   Ideally we should run the next 2 commands in succession for each trajectory.

3. Run the trajectory replay script:
   ```bash
   ros2 run data_collector kuka_data_recorder
   ```
   - Note that the robot will move to the initial position and prompt us for an input to continue.
   - Also, the trajectory replay speed is slower than the data collection rate. We can adjust this if necessary.

4. Once the trajectory has been executed, run the following script to convert joint positions to Cartesian positions:
   ```bash
   ros2 run data_collector joint_cartesian_converter
   ```

5. Repeat the process for additional trajectories:
   - Stop the server.
   - Reset the robot to a non-inserted configuration.
   - Update the JSON file with the name of the next trajectory’s CSV file.

Replayed data is saved in a folder named after the CSV file, and the processed replay data is stored in `recorded_data.csv`.

---

## 4. Converting Trajectories to SERL’s PKL Format
To create a pickle file containing **ALL** trajectories:
1. Run the following script:
   ```bash
   data_collector/data_collector/create_pkl.py
   ```
2. The pickle file will be saved in the folder specified by the **processed_csv_folder_path** variable in `config.json`.