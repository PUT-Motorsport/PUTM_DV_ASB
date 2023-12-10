#ifndef ASB
#define ASB

#include <cstdint>

namespace PUTM_CAN {

enum struct ASB_states : uint8_t {
ASB_OK, // AS ready equivalent
EBS_During_Checkup,
EBS_Checkup_Fail,
ASB_Internal_Warning,
ASB_SCS_IMPLAUSIBLE,
EBS_Emergency_Braking,
EBS_Monitoring_Fail,
};

struct __attribute__((packed)) ASB_main {
  uint16_t airpressure_bank1 : 14;   // 0-10000 [mbar]
  uint16_t airpressure_bank2 : 14;   // 0-10000 [mbar]
  bool SDC_Ready : 1;                // 0 - not ready, 1 - ready
  bool valve1_active : 1;            // 0 - not active, 1 - active
  bool valve2_active : 1;            // 0 - not active, 1 - active
  ASB_states device_state;   // -----------------------------------------------------------------
};

const uint16_t ASB_MAIN_CAN_ID = 0x3b; // 59 decimal
const uint8_t ASB_MAIN_CAN_DLC = sizeof(ASB_main); 
const uint8_t ASB_MAIN_FREQUENCY = 50;   // [Hz]

const CAN_TxHeaderTypeDef can_tx_header_ASB_MAIN{
ASB_MAIN_CAN_ID, 0xFFF, CAN_ID_STD, CAN_RTR_DATA, APPS_MAIN_CAN_DLC, DISABLE};
}   // namespace PUTM_CAN


#endif
