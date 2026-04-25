#include "lq_img_trans_http.hpp"

lq_img_trans_http::lq_img_trans_http() : width_(640), height_(360), fps_(120), port_(8080), frame_delay_opt_(FRAME_LOSS_MODE)
{
}

lq_img_trans_http::~lq_img_trans_http()
{
}
