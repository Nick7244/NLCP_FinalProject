import ur5

def main():
    # Create UR5 Calculator Object
    ur5Calculator = ur5.robot_config('C:/Development/UR5-Box-Sorting-Robot/trajectory_controller/data')

    # Initialize the Mq and Mq_g matricies
    one = ur5Calculator.Mq([0,0,0,0,0,0])
    two = ur5Calculator.Cq([0,0,0,0,0,0], [0,0,0,0,0,0])
    three = ur5Calculator.Mq_g([0,0,0,0,0,0])

if __name__ == "__main__":
    main()
