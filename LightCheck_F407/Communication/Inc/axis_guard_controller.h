#ifndef AXIS_GUARD_CONTROLLER_H
#define AXIS_GUARD_CONTROLLER_H
#include "command_manager.h"
void GuardController_Task(command_manager_t *manager);
void GuardController_Command(command_manager_t *manager, const pc_command_packet_t *command);
pc_status_t GuardController_Preflight(command_manager_t *manager, const pc_command_packet_t *command);
void GuardController_Cancel(command_manager_t *manager, uint8_t axis);
modbus_result_t Command_Manager_RunVelocity(command_manager_t *manager, uint8_t axis, int16_t rpm);
modbus_result_t Command_Manager_StopAxis(command_manager_t *manager, uint8_t axis);
#endif
