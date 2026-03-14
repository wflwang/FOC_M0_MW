/**
 * @file hw_names.h
 * @brief Hardware Board Names and Definitions
 */

#ifndef HW_NAMES_H
#define HW_NAMES_H

/*============================================================================*/
/* Hardware Board Enumeration                                                 */
/*============================================================================*/

typedef enum {
    HW_HK32M070_QFN32 = 100,      // 纠编机主控 - QFN32封装
    HW_HK32M070_QFN48 = 101,      // 扩展版 - QFN48封装
    HW_CUSTOM          = 255,      // 自定义板
} hw_type_e;

/*============================================================================*/
/* Hardware Name Strings                                                      */
/*============================================================================*/

#define HW_NAME_HK32M070_QFN32      "HK32M070_QFN32"
#define HW_NAME_HK32M070_QFN48      "HK32M070_QFN48"
#define HW_NAME_CUSTOM              "CUSTOM"

/*============================================================================*/
/* Firmware Version                                                           */
/*============================================================================*/

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
#define FW_VERSION_PATCH    0

#define FW_VERSION_STRING   "1.0.0"

#endif /* HW_NAMES_H */
