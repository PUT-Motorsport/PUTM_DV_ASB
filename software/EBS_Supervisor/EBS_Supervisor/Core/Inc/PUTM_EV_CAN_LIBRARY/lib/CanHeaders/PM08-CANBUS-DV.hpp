#ifndef DV
#define DV

#include <cstdint>

namespace PUTM_CAN {

enum struct AutonomousSystemStatus : uint8_t {
	Off,
    Ready,
    Driving,
    Emergency,
    Finished,
};

struct __attribute__ ((packed)) DV_Ass{
	AutonomousSystemStatus status;
};


const uint16_t DV_ASS_CAN_ID = 0x07;
const uint8_t DV_ASS_CAN_DLC = sizeof(DV_Ass);
const uint8_t DV_ASS_FREQUENCY = 1;

const CAN_TxHeaderTypeDef can_tx_header_DV_ASS{
DV_ASS_CAN_ID, 0xFFF, CAN_ID_STD, CAN_RTR_DATA, DV_ASS_CAN_DLC, DISABLE};

} // namespace can

#endif