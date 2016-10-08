#ifndef CANSEND_H
#define CANSEND_H


#include "moons/controlcan.h"
#include "moons/applicfg.h"

/**
 * @brief The CAN message structure
 * @ingroup can
 */
typedef struct {
  UNS16 cob_id;	/**< message's ID */
  UNS8 rtr;		/**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
  UNS8 len;		/**< message's length (0 to 8) */
  UNS8 data[8]; /**< message's datas */
} Message;

typedef union {
    struct{
        UNS8 cmd;
        UNS8 indexL;
        UNS8 indexH;
        UNS8 subIndex;
        UNS8 data0;
        UNS8 data1;
        UNS8 data2;
        UNS8 data3;
    }filed;
    UNS8 sdoData[8];
}SDODataType;

typedef union{
	struct{
		UNS8 indexL;
		UNS8 indexH;
		UNS8 subIndex;
	}filed;
}SDOIndex;

class CanSendObj
{
public:
    explicit CanSendObj();
    ~CanSendObj();
    /**
     * @brief Send a CAN message
     * @param port CanFestival file descriptor
     * @param *m The CAN message to send
     * @return 0 if succes
     */
    UNS8 canSend(Message *m);
    UNS8 canReceive_loop();
    UNS8 canReceive();
    UNS8 canReceiveUnpack(VCI_CAN_OBJ &recResponse,SDOIndex sdoIndex,int opIndex);
    ///***************************************************************************/
    UNS8 canSend_driver(Message const *m);
    VCI_CAN_OBJ ReceiveBuf[100];
private:
    int32_t ileftSpeed;
    int32_t iRightSpeed;
};

#endif // CANSEND_H
