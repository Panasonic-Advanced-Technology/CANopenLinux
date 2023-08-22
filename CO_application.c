/*
 * Application interface for CANopenNode.
 *
 * @file        CO_application.c
 * @author      --
 * @copyright   2021 --
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "CO_application.h"
#include <stdlib.h>
#include "OD.h"
#include "OD_2nd.h"

/* status wordのビット定義 */
typedef struct 
{
    uint16_t RTSO:1;    // bit0     Ready to switch on
    uint16_t SO:1;      // bit1     Switch on
    uint16_t OE:1;      // bit2     Operation enabled
    uint16_t F:1;       // bit3     Fault
    uint16_t VE:1;      // bit4     Voltage enabled
    uint16_t QS:1;      // bit5     Quick stop
    uint16_t SOD:1;     // bit6     Switch on disable
    uint16_t W:1;       // bit7     Warning
    uint16_t MS2:1;     // bit8     Manufacture specific
    uint16_t RM:1;      // bit9     Remote
    uint16_t TR:1;      // bit10    Target reached
    uint16_t ILA:1;     // bit11    Internal limit active
    uint16_t OMS:2;     // bit12,13 Operation mode specific
    uint16_t MS:2;      // bit14,15 Manufacture specific
} STATUS_WORD_t;

#define NODE_ID_RIGHT   0xA
#define IDX_RIGHT      0
#define NODE_ID_LEFT    0xB
#define IDX_LEFT       1

STATUS_WORD_t statusword_[2] = {0};
int32_t velocity_[2] = {0};

/* 6041h statusword 受信のコールバック */
ODR_t H6041_write(OD_stream_t *stream, const void *buf,
                   OD_size_t count, OD_size_t *countWritten)
{
    if (stream == NULL || stream->object == NULL || buf == NULL || countWritten == NULL) {
        printf("H6041_write(%x, %x, %x)\n", stream, buf, countWritten);
        return ODR_DEV_INCOMPAT;
    }
    printf("H6041_write(%X) : ", *(uint16_t*)stream->object);

    uint16_t *can_id = (uint16_t*)stream->object;
    uint8_t idx;
    if (*can_id == NODE_ID_RIGHT)
        idx = IDX_RIGHT;
    else if (*can_id == NODE_ID_LEFT)
        idx = IDX_LEFT;
    else
    {
        printf("bad parameter\n");
        return ODR_DEV_INCOMPAT;
    }

    statusword_[idx] = *(STATUS_WORD_t*)buf;

    if (idx == IDX_RIGHT)
        printf("RIGHT = 0x%x ", statusword_[idx]);
    else
        printf("LEFT  = 0x%x ", statusword_[idx]);

    printf("RTSO(%d) SwOn(%d) OpeEn(%d) Fault(%d) SOD(%d) VE(%d)\n", 
        statusword_[idx].RTSO, statusword_[idx].SO, statusword_[idx].OE, statusword_[idx].F, statusword_[idx].SOD, statusword_[idx].VE);

    return ODR_OK;
}
OD_extension_t H6041_extentionR = {
    .object = NULL,
    .read = NULL,
    .write = H6041_write,
};
OD_extension_t H6041_extentionL = {
    .object = NULL,
    .read = NULL,
    .write = H6041_write,
};

/* 6064H Position actual valueのコールバック */
ODR_t H6064_write(OD_stream_t *stream, const void *buf,
                   OD_size_t count, OD_size_t *countWritten)
{
    if (stream == NULL || stream->object == NULL || buf == NULL || countWritten == NULL) {
        printf("H6064_write(%x, %x, %x)\n", stream, buf, countWritten);
        return ODR_DEV_INCOMPAT;
    }
    printf("H6064_write");
    uint16_t *can_id = (uint16_t*)stream->object;
    int32_t pos = *(int32_t*)buf;
    if (*can_id == NODE_ID_RIGHT)
        printf("  posR = %d\n", pos);
    else if (*can_id == NODE_ID_LEFT)
        printf("  posL = %d\n", pos);
    else
        printf("bad parameter\n");

    fflush(stdout);
    return ODR_OK;
}
OD_extension_t H6064_extentionR = {
    .object = NULL,
    .read = NULL,
    .write = H6064_write,
};
OD_extension_t H6064_extentionL = {
    .object = NULL,
    .read = NULL,
    .write = H6064_write,
};

/* 6040h Controlwordのコールバック */
ODR_t H6040_read(OD_stream_t *stream, void *buf,
                   OD_size_t count, OD_size_t *countRead)
{
    if (stream == NULL || stream->object == NULL || buf == NULL || countRead == NULL) {
        printf("H6040_read(%x, %x, %x)\n", stream, buf, countRead);
        return ODR_DEV_INCOMPAT;
    }
    printf("H6040_read(%X): ", *(uint16_t*)stream->object);
    uint16_t *can_id = (uint16_t*)stream->object;
    uint8_t idx;
    if (*can_id == NODE_ID_RIGHT)
        idx = IDX_RIGHT;
    else if (*can_id == NODE_ID_LEFT)
        idx = IDX_LEFT;
    else
    {
        printf("bad parameter\n");
        return ODR_DEV_INCOMPAT;
    }

    if (buf)
    {
        uint16_t* ctrl = (uint16_t*)buf;
        if (statusword_[idx].SO) {
            printf("Openation Enabled\n");
            *ctrl = 0xf;
        } else if (statusword_[idx].RTSO) {
            printf("Send Switch On\n");
            *ctrl = 0x7;
        } else if (statusword_[idx].SOD) {
            printf("Send Ready to switch on\n");
            *ctrl = 0x6;
        }
    }

    return ODR_OK;
}
OD_extension_t H6040_extentionR = {
    .object = NULL,
    .read = H6040_read,
    .write = NULL,
};
OD_extension_t H6040_extentionL = {
    .object = NULL,
    .read = H6040_read,
    .write = NULL,
};

/* 60FFh Target velocityのコールバック */
ODR_t H60FF_read(OD_stream_t *stream, void *buf,
                   OD_size_t count, OD_size_t *countRead)
{
    if (stream == NULL || stream->object == NULL || buf == NULL || countRead == NULL) {
        return ODR_DEV_INCOMPAT;
    }
    printf("H60FF_read(%X) : ", *(uint16_t*)stream->object);
    uint16_t *can_id = (uint16_t*)stream->object;
    uint8_t idx;
    if (*can_id == NODE_ID_RIGHT)
        idx = IDX_RIGHT;
    else if (*can_id == NODE_ID_LEFT)
        idx = IDX_LEFT;
    else
    {
        printf("bad parameter\n");
        return ODR_DEV_INCOMPAT;
    }

    int32_t* ctrl = (int32_t*)buf;
    *ctrl = velocity_[idx];
    printf("set vel %d\n", *ctrl);

    return ODR_OK;
}
OD_extension_t H60FF_extentionR = {
    .object = NULL,
    .read = H60FF_read,
    .write = NULL,
};
OD_extension_t H60FF_extentionL = {
    .object = NULL,
    .read = H60FF_read,
    .write = NULL,
};

uint8_t *OD_targetVel_flagsPDO[2] = {NULL};

/******************************************************************************/
CO_ReturnError_t app_programStart(uint16_t *bitRate,
                                  uint8_t *nodeId,
                                  uint32_t *errInfo)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    //*************************************
    // set sync
    //*************************************
    ODR_t odRet;
    // Set SYNC Period
    odRet = OD_set_u32(OD_ENTRY_H1006_communicationCyclePeriod, 0, 1000000, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1006_communicationCyclePeriod);
        return CO_ERROR_OD_PARAMETERS;
    }
    // Enable SYNC
    odRet = OD_set_u32(OD_ENTRY_H1005_COB_ID_SYNCMessage, 0, 0x40000080, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1005_COB_ID_SYNCMessage);
        return CO_ERROR_OD_PARAMETERS;
    }

    //*************************************
    // Set RPDO for Right Motor
    //*************************************
    odRet = OD_set_u32(OD_ENTRY_H1400_RPDOCommunicationParameter, 1, 0x00000380 | NODE_ID_RIGHT, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1400_RPDOCommunicationParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u8(OD_ENTRY_H1600_RPDOMappingParameter, 0, 2, true); 
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1600_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_ENTRY_H1600_RPDOMappingParameter, 1, 0x60410010, true);               /* Map Statusword */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1600_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_ENTRY_H1600_RPDOMappingParameter, 2, 0x60640020, true);               /* Map Position actual value */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1600_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }

    uint16_t *objR = (uint16_t*)malloc(sizeof(uint16_t));
    *objR = NODE_ID_RIGHT;
    H6041_extentionR.object = objR;
    H6064_extentionR.object = objR;
    OD_extension_init(OD_ENTRY_H6041_statusword, &H6041_extentionR);
    OD_extension_init(OD_ENTRY_H6064_positionActualValue, &H6064_extentionR);

    //*************************************
    // Set TPDO for Right Motor
    //*************************************
    /* Set 1800h TPDOCommunicationParameter*/
    odRet = OD_set_u32(OD_ENTRY_H1800_TPDOCommunicationParameter, 1, 0x00000500 | NODE_ID_RIGHT, true); /* COB-ID */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err : set COB-ID\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u8(OD_ENTRY_H1800_TPDOCommunicationParameter, 2, 0xFE, true); /* Transmission type */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err set Transmission Type\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u16(OD_ENTRY_H1800_TPDOCommunicationParameter, 3, 1000, true); /* Inhibit time*/
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err set Inhibit time\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u16(OD_ENTRY_H1800_TPDOCommunicationParameter, 5, 0, true); /* Event time*/
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err set Event time\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    /* Set 1A00h TPDOMappingParameter */
    odRet = OD_set_u8(OD_ENTRY_H1A00_TPDOMappingParameter, 0, 2, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1A00_TPDOMappingParameter);
        printf("Err2\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_ENTRY_H1A00_TPDOMappingParameter, 1, 0x60400010, true);                   /* Map Controlword */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1A00_TPDOMappingParameter);
        printf("Err3\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_ENTRY_H1A00_TPDOMappingParameter, 2, 0x60FF0020, true);                   /* Map Target velocity */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1A00_TPDOMappingParameter);
        printf("Err4\n");
        return CO_ERROR_OD_PARAMETERS;
    }

    H6040_extentionR.object = objR;
    odRet = OD_extension_init(OD_ENTRY_H6040_controlword, &H6040_extentionR);                       /* Set extention for controlword */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H6040_controlword);
        printf("Err4\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    H60FF_extentionR.object = objR;
    odRet = OD_extension_init(OD_ENTRY_H60FF_targetVelocity, &H60FF_extentionR);                    /* Set extention for target velocity */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H60FF_targetVelocity);
        printf("Err4\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    OD_targetVel_flagsPDO[IDX_RIGHT] = OD_getFlagsPDO(OD_ENTRY_H60FF_targetVelocity);

    //*************************************
    // Set RPDO for Left Motor
    //*************************************
    odRet = OD_set_u32(OD_2nd_ENTRY_H1401_RPDOCommunicationParameter, 1, 0x00000380 | NODE_ID_LEFT, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1401_RPDOCommunicationParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u8(OD_2nd_ENTRY_H1601_RPDOMappingParameter, 0, 2, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1601_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_2nd_ENTRY_H1601_RPDOMappingParameter, 1, 0x60410010, true);               /* Map Statusword */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1601_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_2nd_ENTRY_H1601_RPDOMappingParameter, 2, 0x60640020, true);               /* Map Position actual value */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1601_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }

    uint16_t *objL = (uint16_t*)malloc(sizeof(uint16_t));
    *objL = NODE_ID_LEFT;
    H6041_extentionL.object = objL;
    H6064_extentionL.object = objL;
    OD_extension_init(OD_2nd_ENTRY_H6041_statusword, &H6041_extentionL);
    OD_extension_init(OD_2nd_ENTRY_H6064_positionActualValue, &H6064_extentionL);

    //*************************************
    // Set TPDO for Left Motor
    //*************************************
    /* Set 1800h TPDOCommunicationParameter*/
    odRet = OD_set_u32(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter, 1, 0x00000500 | NODE_ID_LEFT, true); /* COB-ID */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err : set COB-ID\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u8(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter, 2, 0xFE, true); /* Transmission type */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err set Transmission Type\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u16(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter, 3, 1000, true); /* Inhibit time*/
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err set Inhibit time\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u16(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter, 5, 0, true); /* Event time*/
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1800_TPDOCommunicationParameter);
        printf("Err set Event time\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    /* Set 1A00h TPDOMappingParameter */
    odRet = OD_set_u8(OD_2nd_ENTRY_H1A00_TPDOMappingParameter, 0, 2, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1A00_TPDOMappingParameter);
        printf("Err2\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_2nd_ENTRY_H1A00_TPDOMappingParameter, 1, 0x60400010, true);   /* Map Controlworld */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1A00_TPDOMappingParameter);
        printf("Err3\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_2nd_ENTRY_H1A00_TPDOMappingParameter, 2, 0x60FF0020, true);   /* Map Target velocity */
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1A00_TPDOMappingParameter);
        printf("Err4\n");
        return CO_ERROR_OD_PARAMETERS;
    }

    H6040_extentionL.object = objL;
    odRet = OD_extension_init(OD_2nd_ENTRY_H6040_controlword, &H6040_extentionL);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H6040_controlword);
        printf("Err4\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    H60FF_extentionL.object = objL;
    odRet = OD_extension_init(OD_2nd_ENTRY_H60FF_targetVelocity, &H60FF_extentionL);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H60FF_targetVelocity);
        printf("Err4\n");
        return CO_ERROR_OD_PARAMETERS;
    }
    OD_targetVel_flagsPDO[IDX_LEFT] = OD_getFlagsPDO(OD_2nd_ENTRY_H60FF_targetVelocity);

    return err;
}


/******************************************************************************/
void app_communicationReset(CO_t *co) {

    /* example printouts */
    if (!co->nodeIdUnconfigured) {
        /* CANopen Node-ID is configured and all services will work. */
    }
    else {
        /* CANopen Node-ID is unconfigured, so only LSS slave will work. */
    }
}


/******************************************************************************/
void app_programEnd() {
    printf("ProgramEnd\n");
}


/******************************************************************************/
void app_programAsync(CO_t *co, uint32_t timer1usDiff) {
    (void) co; (void) timer1usDiff; /* unused */
    bool val_chagned = false;

    if ( velocity_[IDX_RIGHT] != OD_RAM.x2000_velocityCommand[IDX_RIGHT])
    {
        velocity_[IDX_RIGHT] = OD_RAM.x2000_velocityCommand[IDX_RIGHT];
        printf("change right velocity. ReqestTPDO\n");
        OD_requestTPDO(OD_targetVel_flagsPDO[IDX_RIGHT], 0);
        val_chagned = true;
    }

    if ( velocity_[IDX_LEFT] != OD_RAM.x2000_velocityCommand[IDX_LEFT])
    {
        velocity_[IDX_LEFT] = OD_RAM.x2000_velocityCommand[IDX_LEFT];
        printf("change left velocity. ReqestTPDO\n");
        OD_requestTPDO(OD_targetVel_flagsPDO[IDX_LEFT], 0);
        val_chagned = true;
    }

    if (val_chagned)
        printf("left %d, right %d\n", velocity_[IDX_LEFT], velocity_[IDX_RIGHT]);
}


/******************************************************************************/
void app_programRt(CO_t *co, uint32_t timer1usDiff) {
    (void) co; (void) timer1usDiff; /* unused */
}
