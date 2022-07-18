#include"data_type.h"

DETECTBOX DETECTION_ROW::to_xyah() const
{ //(centerx, centery, ration, h)
    DETECTBOX ret = tlwh;
    ret(0, IDX_X) += (ret(0, IDX_W) * 0.5);
    ret(0, IDX_Y) += (ret(0, IDX_H) * 0.5);
    ret(0, IDX_W) /= ret(0, IDX_H);
    return ret;
}

DETECTBOX DETECTION_ROW::to_tlbr() const
{ //(x,y,xx,yy)
    DETECTBOX ret = tlwh;
    ret(0, IDX_W) += ret(0, IDX_X);
    ret(0, IDX_H) += ret(0, IDX_Y);
    return ret;
}

