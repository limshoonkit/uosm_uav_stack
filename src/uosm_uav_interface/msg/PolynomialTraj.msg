# PolynomialTraj.msg
int16 drone_id
uint64 traj_id
builtin_interfaces/Time start_time

uint8 order
float32[] coef_x
float32[] coef_y
float32[] coef_z
float32[] duration