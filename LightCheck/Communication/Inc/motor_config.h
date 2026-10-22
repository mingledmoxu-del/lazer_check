#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* Both drives: PA72=1152, PA73=0, saved and reset with external SON OFF.
 * Override to 9600U only for drives still configured with PA72=96. */
#ifndef MOTOR_RS485_BAUD
#define MOTOR_RS485_BAUD 115200U
#endif

/* Legacy fixed-two-input prototype (retained for regression tests).
 * Keep 0 when using the new runtime-configurable axis_guard implementation.
 * Runtime DI0..DI5 sampling is always available; its policies default OFF and
 * are explicitly armed through PC_CMD_CONFIG_GUARD, independent of this macro.
 * 0: no LEGACY limit interlock/stop/limit-bypass lease; hardware E-stop required.
 * Continuous rotation's communication watchdog is independent and stays active.
 * 1: restore the retained limit implementation (rebuild and flash required).
 * A compiler definition may override this for regression tests.
 */
#ifndef MOTOR_LIMITS_ENABLED
#define MOTOR_LIMITS_ENABLED 0
#endif

#endif
