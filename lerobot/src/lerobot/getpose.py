# scripts/get_fairino_pose.py
from dataclasses import asdict
from lerobot.robots.fairino_follower.fairino_follower import FairinoFollower, FairinoFollowerConfig

def main():
    config = FairinoFollowerConfig(controller_ip="192.168.0.10")  # update IP as needed
    robot = FairinoFollower(config)
    robot.connect()
    try:
        observation = robot.get_observation()
        print(observation)  # pose data
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()
