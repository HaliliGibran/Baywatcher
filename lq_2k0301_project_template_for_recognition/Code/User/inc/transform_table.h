#ifndef _USER_CAMERA_TRANSFORM_TABLE_H_
#define _USER_CAMERA_TRANSFORM_TABLE_H_

#include "common.h"

// Scope: global read-only table, inverse perspective map Y (pixel coords)
extern const float UndistInverseMapH[BW_RECOG_TRANSFORM_TABLE_HEIGHT][BW_RECOG_TRANSFORM_TABLE_WIDTH];
// Scope: global read-only table, inverse perspective map X (pixel coords)
extern const float UndistInverseMapW[BW_RECOG_TRANSFORM_TABLE_HEIGHT][BW_RECOG_TRANSFORM_TABLE_WIDTH];

#endif /* _USER_CAMERA_TRANSFORM_TABLE_H_ */
