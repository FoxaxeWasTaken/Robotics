"""connect4_play - the console of `tools/so101 play` (docs/project/phase2/P2-02 §6), on the simulation:

    ros2 run so101_bringup connect4_play

so101_connect4's console with the simulated person's hand: the console itself names no sim service.
"""

from so101_bringup.sim_hand import SimHand
from so101_connect4 import play


def main(args=None):
    play.main(SimHand, args)


if __name__ == "__main__":
    main()
