#define LQR_K_NUM               46
#define LQR_MIN_LEN_CTRL       0.15f
#define LQR_MAX_LEN_CTRL       0.35f
#define LQR_LEN_RESOLUTION     0.01f

#define VMC_L1 0.150f
#define VMC_L2 0.270f
#define VMC_MotorDistance 0.150f
#define VMC_HalfMotorDistance (VMC_MotorDistance / 2.0f)

#define LHIP1_OFFSET 0x11FF
#define LHIP2_OFFSET 0x25FF
#define RHIP1_OFFSET 0xC6C3
#define RHIP2_OFFSET 0x8419

#define WHEEL_RADIUS 0.077f

#define MAX_HIP_TOR 40.0f
#define MAX_WHEEL_TOR 15.0f