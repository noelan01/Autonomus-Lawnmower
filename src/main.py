import node_lawnmower_control
drive_node = node_lawnmower_control.Lawnmower_Control()

if __name__ == '__main__':
    for i in range(100):    # send drive commands to Lawnmower_Control node
        drive_node.drive(0.5, 0)
    print("End of main")