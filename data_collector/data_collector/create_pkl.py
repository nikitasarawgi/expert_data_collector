## Format of the pkl file
# The pkl file is a dictionary with the following keys:
# dict(
#         observations=obs,
#         actions=actions,
#         next_observations=next_obs,
#         rewards=rew,
#         masks=1.0 - done,
#         dones=done,
#     )
# where
# observations : {
#     "state": gym.spaces.Box(-np.inf, np.inf, shape=(7 + 6 + 3 + 3,)),
#     "wrist_1": gym.spaces.Box(0, 255, shape=(128, 128, 3), dtype=np.uint8),
#     "wrist_2": gym.spaces.Box(0, 255, shape=(128, 128, 3), dtype=np.uint8),
# }
# actions = gym.spaces.Box(
#             np.ones((7,), dtype=np.float32) * -1,
#             np.ones((7,), dtype=np.float32),
#         )
# state : concatenation of tcp_poses (7), tcp_velocities (6), tcp_forces (3), tcp_torques (3)
# image names are in the format {timestamp}_A.png and {timestamp}_B.png

import numpy as np 
import pandas as pd 
import pickle 
import os 
from PIL import Image

TARGET_POSE = np.array([
    0.5906439143742067,
    0.07771711953459341,
    0.337835826958042,
    3.1099675,
    0.0146619,
    -0.0078615,
])

REWARD_THRESHOLD = np.array([0.01, 0.01, 0.01, 0.2, 0.2, 0.2])

def convertToPickle(recorded_data_file_path, imageA_folder_path, imageB_folder_path, output_pkl_file_path):

    recorded_data = pd.read_csv(recorded_data_file_path)

    observations = []
    actions = []
    next_observations = []
    rewards = []
    masks = []
    dones = []

    done = 0.0

    for i in range(len(recorded_data) - 1):

        current_row = recorded_data.iloc[i]
        next_row = recorded_data.iloc[i + 1]

        curr_time = current_row["timestamp"]
        next_time = next_row["timestamp"]

        curr_wrist_1_image = Image.open(os.path.join(imageA_folder_path, f"{curr_time}_A.jpg"))
        curr_wrist_2_image = Image.open(os.path.join(imageB_folder_path, f"{curr_time}_B.jpg"))

        next_wrist_1_image = Image.open(os.path.join(imageA_folder_path, f"{next_time}_A.jpg"))
        next_wrist_2_image = Image.open(os.path.join(imageB_folder_path, f"{next_time}_B.jpg"))

        observation = {
            "state": np.concatenate([
                current_row[['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']].values,
                current_row[['V1', 'V2', 'V3', 'V4', 'V5', 'V6']].values,
                current_row[['Fx', 'Fy', 'Fz']].values,
                current_row[['Tx', 'Ty', 'Tz']].values
            ]),
            "wrist_1": np.array(curr_wrist_1_image),
            "wrist_2": np.array(curr_wrist_2_image)
        }

        action = next_row[['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']].values - current_row[['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7']].values

        next_observation = {
            "state": np.concatenate([
                next_row[['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']].values,
                next_row[['V1', 'V2', 'V3', 'V4', 'V5', 'V6']].values,
                next_row[['Fx', 'Fy', 'Fz']].values,
                next_row[['Tx', 'Ty', 'Tz']].values
            ]),
            "wrist_1": np.array(next_wrist_1_image),
            "wrist_2": np.array(next_wrist_2_image)
        }

        if done != 1.0:
            tcp_pose = current_row[['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']].values
            if np.all(np.abs(tcp_pose - TARGET_POSE) <= REWARD_THRESHOLD):
                reward = 1.0
                done = 1.0
            else:
                reward = 0.0
        else:
            reward = 0.0

        observations.append(observation)
        actions.append(action)
        next_observations.append(next_observation)
        rewards.append(reward)
        masks.append(1.0 - done)
        dones.append(done)

        curr_wrist_1_image.close()
        curr_wrist_2_image.close()
        next_wrist_1_image.close()
        next_wrist_2_image.close()


    data_dict = {
        "observations": observations,
        "actions": actions,
        "next_observations": next_observations,
        "rewards": rewards,
        "masks": masks,
        "dones": dones
    }

    with open(output_pkl_file_path, 'wb') as f:
        pickle.dump(data_dict, f)

    print(f"Data saved to {output_pkl_file_path}")

def main():
    recorded_data_file_path = "/home/omey/nisara/expert_data_collector/processed_replay/processed_replay/2013-01-01_01-09-43/recorded_data.csv"
    imageA_folder_path = "/home/omey/nisara/expert_data_collector/processed_replay/processed_replay/2013-01-01_01-09-43/imageA"
    imageB_folder_path = "/home/omey/nisara/expert_data_collector/processed_replay/processed_replay/2013-01-01_01-09-43/imageB"
    output_pkl_file_path = "/home/omey/nisara/expert_data_collector/processed_replay/processed_replay/2013-01-01_01-09-43/recorded_data.pkl"

    convertToPickle(recorded_data_file_path, imageA_folder_path, imageB_folder_path, output_pkl_file_path)

if __name__ == "__main__":
    main()

