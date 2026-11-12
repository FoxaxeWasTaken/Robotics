# The Connect Four grid as it is read (phase2/P2-02 §6).
uint8 EMPTY=0
uint8 HUMAN=1
uint8 ROBOT=2
std_msgs/Header header
int8[42] cells  # row-major from the bottom row: cells[row * 7 + column], column 0 nearest the arm
bool valid      # false: the grid was not read; reason says why
string reason
