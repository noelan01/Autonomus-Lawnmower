import node_lawnmower_control
drive_node = node_lawnmower_control.Lawnmower_Control()


def main()

    # TODO
    """
        Kalla på funktioner som sköter styrningen och låt sedan main kalla på
        drive_node.drive() som skickar in styrsignalerna till motorerna
    """
    while True:    # send drive commands to Lawnmower_Control node
        drive_node.drive(0.5, 0.0)
    print("End of main")


if __name__ == '__main__':
    main()