// Per-joint state for the CiA 402 custom homing module.
//
// The maintained cia402 component owns the drive controlword and operation
// mode.  This module owns only the LinuxCNC homing state, the home request,
// and the enable/feedback pass-throughs needed during drive-owned motion.

#include "lcec_hal_compat.h"

typedef enum {
    CIA_HOME_IDLE = 0,
    CIA_HOME_WAIT_MODE,
    CIA_HOME_RUNNING,
    CIA_HOME_BASE
} cia_home_state_t;

typedef struct {
    // Custom-homing selection.
    hal_bit_t *request_custom_homing;  // IN
    hal_bit_t *is_custom_homing;       // OUT

    // Enable pass-through.  Abort revokes enable until an explicit retry.
    hal_bit_t *cia_enable_in;          // IN
    hal_bit_t *cia_enable_out;         // OUT

    // Commands are sampled, never rewritten from old feedback.
    hal_float_t *cia_pos_cmd;          // IN
    hal_float_t *cia_velocity_cmd;     // IN

    // Actual feedback from cia402.
    hal_float_t *cia_pos_fb_in;        // IN
    hal_float_t *cia_velocity_fb_in;   // IN

    // Feedback presented to LinuxCNC/PID.
    hal_float_t *cia_pos_fb_out;       // OUT
    hal_float_t *cia_velocity_fb_out;  // OUT

    // cia402 drive-homing interface.
    hal_bit_t *cia_home;               // IO; cia402 clears on homed edge
    hal_bit_t *cia_stat_homed;         // IN
    hal_bit_t *cia_stat_homing;        // IN
    hal_bit_t *cia_stat_homing_error;  // IN
    hal_bit_t *cia_stat_fault;         // IN
    hal_s32_t *cia_opmode_display;     // IN
    hal_s32_t *cia_state;              // OUT

    // Zero disables the timeout.  The default is applied at pin creation.
    hal_u32_t *cia_timeout_ms;         // IN

    cia_home_state_t state;
    int64_t elapsed_ns;
    bool enable_inhibit;
} cia402_joint_t;
