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

ODR_t H6064_write(OD_stream_t *stream, const void *buf,
                   OD_size_t count, OD_size_t *countWritten)
{
    printf("H6064_write");
    if (stream == NULL || stream->object == NULL || buf == NULL || countWritten == NULL) {
        printf("\n");
        return ODR_DEV_INCOMPAT;
    }
    uint16_t *can_id = (uint16_t*)stream->object;
    int32_t pos = *(int32_t*)buf;
    if (can_id && *can_id == 0x038A)
        printf("  posR = %d\n", pos);
    else if (can_id && *can_id == 0x038B)
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
    odRet = OD_set_u32(OD_ENTRY_H1006_communicationCyclePeriod, 0, 100000, true);
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
    odRet = OD_set_u32(OD_ENTRY_H1400_RPDOCommunicationParameter, 1, 0x0000038A, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1400_RPDOCommunicationParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u8(OD_ENTRY_H1600_RPDOMappingParameter, 0, 2, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1600_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_ENTRY_H1600_RPDOMappingParameter, 1, 0x60410010, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1600_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_ENTRY_H1600_RPDOMappingParameter, 2, 0x60640020, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_ENTRY_H1600_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }

    uint16_t *objR = (uint16_t*)malloc(sizeof(uint16_t));
    *objR = 0x038A;
    H6064_extentionR.object = objR;
    OD_extension_init(OD_ENTRY_H6064_positionActualValue, &H6064_extentionR);


    //*************************************
    // Set RPDO for Left Motor
    //*************************************
    odRet = OD_set_u32(OD_2nd_ENTRY_H1401_RPDOCommunicationParameter, 1, 0x0000038B, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1401_RPDOCommunicationParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u8(OD_2nd_ENTRY_H1601_RPDOMappingParameter, 0, 2, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1601_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_2nd_ENTRY_H1601_RPDOMappingParameter, 1, 0x60410010, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1601_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }
    odRet = OD_set_u32(OD_2nd_ENTRY_H1601_RPDOMappingParameter, 2, 0x60640020, true);
    if (odRet != ODR_OK) {
        if (errInfo != NULL) *errInfo = OD_getIndex(OD_2nd_ENTRY_H1601_RPDOMappingParameter);
        return CO_ERROR_OD_PARAMETERS;
    }

    uint16_t *objL = (uint16_t*)malloc(sizeof(uint16_t));
    *objL = 0x038B;
    H6064_extentionL.object = objL;
    OD_extension_init(OD_2nd_ENTRY_H6064_positionActualValue, &H6064_extentionL);

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

}


/******************************************************************************/
void app_programAsync(CO_t *co, uint32_t timer1usDiff) {
    (void) co; (void) timer1usDiff; /* unused */
}


/******************************************************************************/
void app_programRt(CO_t *co, uint32_t timer1usDiff) {
    (void) co; (void) timer1usDiff; /* unused */
#if 0
    /* Simulation: detect change of state of the variable and trigger TPDO, to
     * which variable is possibly mapped. In our example x2110_variableInt32[0]
     * variable will be mapped to TPDO. If program detects change-of-state of
     * its value, TPDO will be triggered for sending. (x2110_variableInt32[0]
     * variable can be changed by SDO.)
     */

    /* static variable is like global: initialized to 0, then keeps its value */
    static int32_t value_old = 0;
    int32_t value_current = 0; //OD_RAM.x2110_variableInt32[0];

    /* Detect change of state and trigger TPDO. Of course, variable must be
     * mapped to event driven TPDO for this to have effect. */
    if (value_current != value_old) {
        OD_requestTPDO(OD_variableInt32_flagsPDO, 1); /* subindex is 1 */
    }
    value_old = value_current;
#endif
}
