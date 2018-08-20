/*
 *  Copyright (c) 2012 The WebM project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

//  This is an example demonstrating how to implement a multi-layer VPx
//  encoding scheme based on temporal scalability for video applications
//  that benefit from a scalable bitstream.

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "./vpx_config.h"
#include "../vpx_ports/vpx_timer.h"
#include "vpx/vp8cx.h"
#include "vpx/vpx_encoder.h"
#include "vpx_ports/bitops.h"

#include "../tools_common.h"
#include "../video_writer.h"

extern "C"{
    #include "vpx/vpx_encoder.h"
    #include "vpx/vp8cx.h"
    #include "vpx/vpx_decoder.h"
    #include "vpx/vp8dx.h"
}
#include "SDLFramework.hpp"


#define odbgd(FMT, ARGS...) do{  printf("|%7s|D| " FMT, "main", ##ARGS); printf("\n"); fflush(stdout); }while(0)
#define odbgi(FMT, ARGS...) do{  printf("|%7s|I| " FMT, "main", ##ARGS); printf("\n"); fflush(stdout); }while(0)
#define odbge(FMT, ARGS...) do{  printf("|%7s|E| " FMT, "main", ##ARGS); printf("\n"); fflush(stdout); }while(0)

class VP8LayerStrategy{
    int numTLayers_ = 1;
    int flagsOfTLayer_[16];
    int flagsOfTLayerLength_ = 0;
    int flagIndex_ = -1;
    int layerIdIndex_ = -1;
    
    void config2Layers8FramePeriod1(vpx_codec_enc_cfg_t &enccfg, int bitrateKbps){
        numTLayers_ = 2;
        enccfg.ts_number_layers = numTLayers_;
        enccfg.ts_periodicity = numTLayers_;
        int br = bitrateKbps / numTLayers_;
        for(int i = 0; i < numTLayers_; ++i){
            enccfg.ts_target_bitrate[i] = (i+1)*br;
            enccfg.ts_layer_id[i] = i;
            enccfg.ts_rate_decimator[i] = numTLayers_-i;
        }
        flagsOfTLayerLength_ = 8;
        flagsOfTLayer_[0] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF; // kTemporalUpdateLastAndGoldenRefAltRef;
        flagsOfTLayer_[1] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_LAST; // kTemporalUpdateGoldenWithoutDependencyRefAltRef;
        flagsOfTLayer_[2] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_GF ; // kTemporalUpdateLastRefAltRef;
        flagsOfTLayer_[3] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST; //kTemporalUpdateGoldenRefAltRef;
        flagsOfTLayer_[4] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_REF_GF; // kTemporalUpdateLastRefAltRef;
        flagsOfTLayer_[5] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST; //kTemporalUpdateGoldenRefAltRef;
        flagsOfTLayer_[6] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_REF_GF; // kTemporalUpdateLastRefAltRef;
        flagsOfTLayer_[7] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_GF  | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ENTROPY; // kTemporalUpdateNone;
        
    }
    
    void config2Layers8FramePeriod2(vpx_codec_enc_cfg_t &enccfg, int bitrateKbps){
        numTLayers_ = 2;
        enccfg.ts_number_layers = numTLayers_;
        enccfg.ts_periodicity = numTLayers_;
        int br = bitrateKbps / numTLayers_;
        for(int i = 0; i < numTLayers_; ++i){
            enccfg.ts_target_bitrate[i] = (i+1)*br;
            enccfg.ts_layer_id[i] = i;
            enccfg.ts_rate_decimator[i] = numTLayers_-i;
//            enccfg.ts_rate_decimator[i] = i+1;
        }
        
        
        flagsOfTLayerLength_ = 8;
        
        // Layer 0: predict from L and ARF, update L and G.
        flagsOfTLayer_[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_ARF;
        // Layer 1: sync point: predict from L and ARF, and update G.
        flagsOfTLayer_[1] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ARF;
        // Layer 0, predict from L and ARF, update L.
        flagsOfTLayer_[2] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF;
        // Layer 1: predict from L, G and ARF, and update G.
        flagsOfTLayer_[3] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ENTROPY;
        // Layer 0.
        flagsOfTLayer_[4] = flagsOfTLayer_[2];
        // Layer 1.
        flagsOfTLayer_[5] = flagsOfTLayer_[3];
        // Layer 0.
        flagsOfTLayer_[6] = flagsOfTLayer_[4];
        // Layer 1.
        flagsOfTLayer_[7] = flagsOfTLayer_[5];
    }
    
    void config2Layers2FramePeriod(vpx_codec_enc_cfg_t &enccfg, int bitrateKbps){
        numTLayers_ = 2;
        enccfg.ts_number_layers = numTLayers_;
        enccfg.ts_periodicity = numTLayers_;
        int br = bitrateKbps / numTLayers_;
        for(int i = 0; i < numTLayers_; ++i){
            enccfg.ts_target_bitrate[i] = (i+1)*br;
            enccfg.ts_layer_id[i] = i;
            enccfg.ts_rate_decimator[i] = numTLayers_-i;
        }
        enccfg.ts_target_bitrate[0] = bitrateKbps * 0.6f;
        enccfg.ts_target_bitrate[1] = bitrateKbps;
        enccfg.ts_rate_decimator[0] = 2;
        enccfg.ts_rate_decimator[1] = 1;
        
        flagsOfTLayerLength_ = 2;
        
        //flagsOfTLayer_[0] |= VPX_EFLAG_FORCE_KF;
        flagsOfTLayer_[0] |= (VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF );
        //flagsOfTLayer_[0] |= VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF;
        
        
        flagsOfTLayer_[1] |= (VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_REF_ARF);
        
    }
    
    
    
public:
    VP8LayerStrategy(){
    }
    
    virtual ~VP8LayerStrategy(){
        
    }
    
    void reset(){
        numTLayers_ = 1;
        flagsOfTLayerLength_ = 0;
        flagIndex_ = -1;
        layerIdIndex_ = -1;
        memset(flagsOfTLayer_, 0, sizeof(flagsOfTLayer_));
    }
    
    void configTLayers(vpx_codec_enc_cfg_t &enccfg, int bitrateKbps){
        this->reset();
        //        this->config2Layers8FramePeriod2(enccfg, bitrateKbps);
        this->config2Layers2FramePeriod(enccfg, bitrateKbps);
        
        enccfg.g_error_resilient = 1;
        //enccfg.rc_dropframe_thresh = (unsigned int)strtoul(argv[9], NULL, 0);
        enccfg.rc_end_usage = VPX_CBR;
        enccfg.rc_min_quantizer = 2;
        enccfg.rc_max_quantizer = 56;
        //enccfg.rc_max_quantizer = 52; // VP9
        enccfg.rc_undershoot_pct = 50;
        enccfg.rc_overshoot_pct = 50;
        enccfg.rc_buf_initial_sz = 600;
        enccfg.rc_buf_optimal_sz = 600;
        enccfg.rc_buf_sz = 1000;
        enccfg.rc_resize_allowed = 0;
        enccfg.g_threads = 1;
        enccfg.g_lag_in_frames = 0;
        enccfg.kf_mode = VPX_KF_AUTO;
        enccfg.kf_min_dist = enccfg.kf_max_dist = 3000;
        enccfg.temporal_layering_mode = VP9E_TEMPORAL_LAYERING_MODE_BYPASS;
        enccfg.rc_target_bitrate = bitrateKbps;
        
        enccfg.rc_end_usage = VPX_CBR;
        enccfg.rc_dropframe_thresh = 30;
        enccfg.rc_min_quantizer = 2;
        enccfg.rc_max_quantizer = 56;
        enccfg.rc_undershoot_pct = 15;
        enccfg.rc_overshoot_pct = 15;
        enccfg.rc_buf_initial_sz = 500;
        enccfg.rc_buf_optimal_sz = 600;
        enccfg.rc_buf_sz = 1000;
    }
    
    void configOne(vpx_codec_enc_cfg_t &enccfg, int bitrateKbps){
        this->reset();
        enccfg.rc_end_usage = VPX_CBR;
        enccfg.rc_dropframe_thresh = 30;
        enccfg.rc_min_quantizer = 2;
        enccfg.rc_max_quantizer = 56;
        enccfg.rc_undershoot_pct = 15;
        enccfg.rc_overshoot_pct = 15;
        enccfg.rc_buf_initial_sz = 500;
        enccfg.rc_buf_optimal_sz = 600;
        enccfg.rc_buf_sz = 1000;
    }
    
    void config(vpx_codec_enc_cfg_t &enccfg, int bitrateKbps){
        this->configTLayers(enccfg, bitrateKbps);
//        this->configOne(enccfg, bitrateKbps);
    }
    
    void next(vpx_enc_frame_flags_t &encflags, int &layerId){
        ++layerIdIndex_;
        layerIdIndex_ = layerIdIndex_ % numTLayers_;
        layerId = layerIdIndex_;
        
        ++flagIndex_;
        flagIndex_ = flagIndex_ % flagsOfTLayerLength_;
        encflags |= (this->flagsOfTLayer_[flagIndex_]);
    }
    
    int layerId(){
        return layerIdIndex_;
    }
    
};


class VP8Encoder{
    
protected:
    vpx_codec_ctx_t encctx_;
    vpx_codec_enc_cfg_t enccfg_;
    int width_ = 0;
    int height_ = 0;
    int framerate_ = 0;
    int bitrateKbps_ = 0;
    uint8_t * encodedBuf_ = NULL;
    size_t encodedBufSize_ = 0;
    size_t frameLength_ = 0;
    vpx_codec_frame_flags_t frameFlags_ = 0;
    int numFrames_ = 0;
    int flagsOfTLayer_[16];
    int flagsOfTLayerLength_ = 0;
    int flagIndex_ = 0;
    
    VP8LayerStrategy layerStrategy_;
    
    void freeEncodeBuf(){
        if(encodedBuf_){
            delete []encodedBuf_;
            encodedBuf_ = NULL;
        }
        frameLength_ = 0;
        encodedBufSize_ = 0;
    }
    
    void appendEncodedData(void *data, size_t length){
        if(length == 0) return;
        if((frameLength_ + length) > encodedBufSize_){
            size_t frmlen = frameLength_;
            size_t bufsz = frmlen + length;
            size_t blocksz = 64*1024;
            bufsz = ((bufsz + blocksz -1)/blocksz) * blocksz; // align to blocksz
            uint8_t * buf = new uint8_t[bufsz];
            if(frmlen > 0){
                memcpy(buf, encodedBuf_, frmlen);
            }
            this->freeEncodeBuf();
            encodedBuf_ = buf;
            encodedBufSize_ = bufsz;
            frameLength_ = frmlen;
        }
        memcpy(encodedBuf_+frameLength_, data, length);
        frameLength_ += length;
    }
    
public:
    VP8Encoder(){
        memset(&encctx_, 0, sizeof(encctx_));
        encctx_.name = NULL;
    }
    
    virtual ~VP8Encoder(){
        this->close();
    }
    
    int open(int width, int height, int framerate, int bitrateKbps, int numTLayers = 1){
        
        vpx_codec_flags_t encflags = 0;
        vpx_codec_err_t vpxret = VPX_CODEC_OK;
        int ret = 0;
        
        do{
            this->close();
            if(numTLayers < 1){
                numTLayers = 1;
            }
            width_ = width;
            height_ = height;
            bitrateKbps_ = bitrateKbps;
            framerate_ = framerate;
            
            vpxret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), &enccfg_, 0);
            enccfg_.g_w = width_;
            enccfg_.g_h = height_;
            enccfg_.rc_target_bitrate = bitrateKbps_; // in unit of kbps
//            enccfg_.kf_mode = VPX_KF_AUTO;
            
            layerStrategy_.config(enccfg_, bitrateKbps);
            

//            encflags |= VPX_CODEC_USE_OUTPUT_PARTITION;
            vpxret = vpx_codec_enc_init_ver(&encctx_,
                                            vpx_codec_vp8_cx(),
                                            &enccfg_,
                                            encflags, VPX_ENCODER_ABI_VERSION);
            if(vpxret != VPX_CODEC_OK){
                odbge("vpx_codec_enc_init error with %d-[%s]", vpxret, vpx_codec_error(&encctx_));
                ret = -1;
                encctx_.name = NULL;
                break;
            }
            
            // for VP8 
            //vpx_codec_control(&encctx_, VP8E_SET_CPUUSED, -speed);
            vpx_codec_control(&encctx_, VP8E_SET_NOISE_SENSITIVITY, 0);
            vpx_codec_control(&encctx_, VP8E_SET_STATIC_THRESHOLD, 1);
            vpx_codec_control(&encctx_, VP8E_SET_GF_CBR_BOOST_PCT, 0);
            
            
            ret = 0;
        }while(0);
        
        if(ret){
            close();
        }
        
        return ret;
    }
    
    void close(){
        if(encctx_.name){
            vpx_codec_destroy(&encctx_);
            encctx_.name = NULL;
        }
        this->freeEncodeBuf();
        numFrames_ = 0;
        flagsOfTLayerLength_ = 0;
        flagIndex_ = 0;
    }
    
    bool isTLayerEnabled(){
        bool e = (enccfg_.ts_number_layers <= 1 || enccfg_.ts_periodicity == 0);
        return !e;
    }
    int getTLayerId(){
        return layerStrategy_.layerId();
    }
    
    
    int encode(vpx_image_t * vpximg, vpx_codec_pts_t pts, uint32_t duration, vpx_enc_frame_flags_t encflags = 0, bool forceKeyframe = false){
        vpx_codec_err_t vpxret = VPX_CODEC_OK;
        int ret = -1;
        do{
            frameLength_ = 0;
            frameFlags_ = 0;
            
            //uint32_t duration = 90000 / framerate_;
            //vpx_enc_frame_flags_t encflags = 0;
            if (forceKeyframe){
                encflags |= VPX_EFLAG_FORCE_KF;
            }
            if(this->isTLayerEnabled()){
                int layerId = 0;
                layerStrategy_.next(encflags, layerId);
                vpx_codec_control(&encctx_, VP8E_SET_TEMPORAL_LAYER_ID, layerId);
            }
            
            vpxret = vpx_codec_encode(&encctx_, vpximg, pts, 1, encflags, VPX_DL_REALTIME);
            if (vpxret != VPX_CODEC_OK) {
                odbge("vpx_codec_encode fail with %d", vpxret);
                ret = -1;
                break;
            }
            
            vpx_codec_iter_t iter = NULL;
            const vpx_codec_cx_pkt_t *pkt = NULL;
            int npkts = 0;
            while ((pkt = vpx_codec_get_cx_data(&encctx_, &iter)) != NULL) {
                if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
                    const int keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
                    odbgi("  pkt[%d]: pts=%lld, data=%p, sz=%zu, part=%d, k=%d"
                          , npkts
                          , pkt->data.frame.pts
                          , pkt->data.frame.buf
                          , pkt->data.frame.sz
                          , pkt->data.frame.partition_id
                          , keyframe);
                    frameFlags_ = pkt->data.frame.flags;
                    this->appendEncodedData(pkt->data.frame.buf, pkt->data.frame.sz);

                    ++npkts;
                }else{
                    odbgi("  non-frame-pkt, kind=%d", pkt->kind);
                }
            }
            if(npkts > 0){
                ++numFrames_;
            }
            
            ret = 0;
        }while(0);
        return ret;
    }
    
    uint8_t * getEncodedFrame(){
        return this->encodedBuf_;
    }
    
    size_t getEncodedLength(){
        return frameLength_;
    }
    
    vpx_codec_frame_flags_t getFrameFlags(){
        return frameFlags_;
    }
};

class VP8Decoder{
    
protected:
    vpx_codec_ctx_t decctx_;
    vpx_codec_iter_t imageIter_ = NULL;
public:
    VP8Decoder(){
        memset(&decctx_, 0, sizeof(decctx_));
        decctx_.name = NULL;
    }
    
    virtual ~VP8Decoder(){
        this->close();
    }
    
    int open(){
        vpx_codec_err_t vpxret = VPX_CODEC_OK;
        int ret = 0;
        do{
            this->close();
            
            vpx_codec_dec_cfg_t deccfg;
            deccfg.threads = 1;
            deccfg.h = deccfg.w = 0;  // set after decode
            vpx_codec_flags_t decflags = 0;
            //        decflags = VPX_CODEC_USE_POSTPROC;
            //        decflags |= VPX_CODEC_USE_INPUT_PARTITION;
            //        decflags |= VPX_CODEC_USE_INPUT_FRAGMENTS;
            
            vpxret = vpx_codec_dec_init_ver(&decctx_, vpx_codec_vp8_dx(), &deccfg, decflags, VPX_DECODER_ABI_VERSION);
            if(vpxret != VPX_CODEC_OK){
                ret = -1;
                decctx_.name = NULL;
                break;
            }
            ret = 0;
        }while(0);
        
        if(ret){
            close();
        }
        
        return ret;
    }
    
    void close(){
        if(decctx_.name){
            vpx_codec_destroy(&decctx_);
            decctx_.name = NULL;
        }
        imageIter_ = NULL;
    }
    
    
    int decode(const uint8_t * frameData, size_t frameLength
               , int * pRefUpdate = NULL
               , int * pCorrupted = NULL
               , int * pRefUsed = NULL){
        imageIter_ = NULL;
        vpx_codec_err_t vpxret = vpx_codec_decode(&decctx_, frameData, (unsigned int)frameLength, NULL, 0);
        if (vpxret != VPX_CODEC_OK) {
            odbge("vpx_codec_decode fail with %d", vpxret);
            return -1;
        }
        
        int refused = 0;
        vpxret = vpx_codec_control(&decctx_, VP8D_GET_LAST_REF_USED, &refused);
        if (vpxret != VPX_CODEC_OK) {
            odbge("vpx_codec_control VP8D_GET_LAST_REF_USED fail with %d", vpxret);
        }
        
        int refupdates = 0;
        vpxret = vpx_codec_control(&decctx_, VP8D_GET_LAST_REF_UPDATES, &refupdates);
        if (vpxret != VPX_CODEC_OK) {
            odbge("vpx_codec_control VP8D_GET_LAST_REF_UPDATES fail with %d", vpxret);
        }
        
        int corrupted = 0;
        vpxret = vpx_codec_control(&decctx_, VP8D_GET_FRAME_CORRUPTED, &corrupted);
        if (vpxret != VPX_CODEC_OK) {
            odbge("vpx_codec_control VP8D_GET_FRAME_CORRUPTED fail with %d", vpxret);
        }
        //odbgi("decode: ref=0x%02x, corrupted=%d", refupdates, corrupted);
        
        if(pRefUpdate){
            *pRefUpdate = refupdates;
        }
        if(pCorrupted){
            *pCorrupted = corrupted;
        }
        if(pRefUsed){
            *pRefUsed = refused;
        }
        
        return 0;
    }
    
    vpx_image_t * pullImage(){
        vpx_image_t * decimg = vpx_codec_get_frame(&decctx_, &imageIter_);
        return decimg;
    }
};



// static
// int vpx_img_plane_width(const vpx_image_t *img, int plane) {
//     if (plane > 0 && img->x_chroma_shift > 0)
//         return (img->d_w + 1) >> img->x_chroma_shift;
//     else
//         return img->d_w;
// }

// static
// int vpx_img_plane_height(const vpx_image_t *img, int plane) {
//     if (plane > 0 && img->y_chroma_shift > 0)
//         return (img->d_h + 1) >> img->y_chroma_shift;
//     else
//         return img->d_h;
// }

// static
// int vpx_img_read(vpx_image_t *img, FILE *file) {
//     int factor = ((img->fmt & VPX_IMG_FMT_HIGHBITDEPTH) ? 2 : 1);
//     for (int plane = 0; plane < 3; ++plane) {
//         unsigned char *buf = img->planes[plane];
//         const int stride = img->stride[plane];
//         const int w = vpx_img_plane_width(img, plane) * factor;
//         const int h = vpx_img_plane_height(img, plane);
//         int y;
        
//         for (y = 0; y < h; ++y) {
//             if (fread(buf, 1, w, file) != (size_t)w){
//                 return -1;
//             }
//             buf += stride;
//         }
//     }
    
//     return 0;
// }

static
void dump_vp8_frame(uint8_t * frameData, size_t frameSize){
    uint8_t firstByte = frameData[0];
    int frmtype = (firstByte >> 0 & 0x1);
    int ver = firstByte >> 1 & 0x7;
    int disp = (firstByte >> 4 & 0x1) ;
    int part1Len = (((firstByte >> 5) & 0x7) << 0) | (frameData[1] << 3) | (frameData[2] << 11);
    odbgi("  frame: type=%d, ver=%d, disp=%d, part1=%d", frmtype, ver, disp, part1Len);
    if(frmtype == 0){
        // startcode=[9d 01 2a]
        // Horizontal 16 bits      :     (2 bits Horizontal Scale << 14) | Width (14 bits)
        // Vertical   16 bits      :     (2 bits Vertical Scale << 14) | Height (14 bits)
        uint8_t * ptr = frameData + 3;
        uint16_t wordH = ptr[4] << 8 | ptr[3];
        uint16_t wordV = ptr[6] << 8 | ptr[5];
        int scaleH = (wordH >> 14) & 0x3;
        int width = wordH & 0x3FFF;
        int scaleV = (wordV >> 14) & 0x3;
        int height = wordV & 0x3FFF;
        
        uint8_t * part1Data = (frameData+10);
        int color = part1Data[0] & 0x1;
        int pixel = (part1Data[0]>>1) & 0x1;
        
        odbgi("         startcode=[%02x %02x %02x], horiz=[%d, %d], vert=[%d, %d], color=%d, pixel=%d"
              , ptr[0], ptr[1], ptr[2]
              , scaleH, width, scaleV, height
              , color, pixel);
    }
    
}


static
int makeTextCommon(int encodedLength, vpx_codec_frame_flags_t frameFlags, int numFrames, char * txt){
    if(encodedLength > 0){
        bool isKey = (frameFlags &VPX_FRAME_IS_KEY)!=0;
        return sprintf(txt, " Frame %d, type=%c", numFrames, isKey?'I':'P');
    }else{
        return sprintf(txt, " Frame %d, drop=1", numFrames);
    }
    
}

struct User{
    SDLWindowImpl * window_ = NULL;
    SDLVideoView * videoView_ = NULL;
    SDLTextView * frameTxtView_ = NULL;
    User(const std::string& name, int w, int h)
    : window_(new SDLWindowImpl(name, w, h))
    , videoView_(new SDLVideoView())
    , frameTxtView_(new SDLTextView()){
        window_->addView(videoView_);
        window_->addView(frameTxtView_);
    }
    virtual ~User(){
        if(window_){
            delete window_;
            window_ = NULL;
        }
    }
};

struct EncodeUser : public User{
    VP8Encoder * encoder_ = NULL;
    EncodeUser(const std::string& name, int w, int h)
    : User(name, w, h)
    , encoder_(new VP8Encoder()){
        
    }
    virtual ~EncodeUser(){
        if(encoder_){
            delete encoder_;
            encoder_ = NULL;
        }
    }
};

struct DecodeUser : public User {
    VP8Decoder * decoder_ = NULL;
    int layerId_ = 0;
    int numFrames_ = 0;
    vpx_codec_pts_t lastPTS4FPS_ = 0;
    int lastNumFrames_  =0;
    int fps_ = 0;
    int64_t inputBytes_ = 0;
    int64_t bitrate_ = 0;
    DecodeUser(const std::string& name, int w, int h, int layerId = 0)
    : User(name, w, h)
    , decoder_(new VP8Decoder())
    , layerId_(layerId){
        
    }
    virtual ~DecodeUser(){
        if(decoder_){
            delete decoder_;
            decoder_ = NULL;
        }
    }
    
    void decode(vpx_codec_pts_t pts, uint8_t * encodedFrame, int encodedLength, vpx_codec_frame_flags_t frameFlags
      , int frameNo, int layerId, bool skip){

        if(layerId > layerId_){
            return ;
        }
        
        if(pts < lastPTS4FPS_){
            lastPTS4FPS_ = pts;
            lastNumFrames_ = 0;
            fps_ = 0;
            numFrames_ = 0;
            inputBytes_ = 0;
            bitrate_ = 0;
        }
        
        int ret = 0;
        int refupdate = 0;
        int corrupted = 0;
        int refUsed = 0;
        const uint8_t * frameData = NULL;
        size_t frameLength = 0;
        if(!skip){
            frameData = encodedFrame;
            frameLength = encodedLength;
        }
        inputBytes_ += frameLength;
        ret = decoder_->decode(frameData, frameLength, &refupdate, &corrupted, &refUsed);
        if (ret == 0) {
            vpx_image_t * decimg = NULL;
            while ((decimg = decoder_->pullImage()) != NULL) {
                ++numFrames_;
                vpx_codec_pts_t duration = pts - lastPTS4FPS_;
                int num = numFrames_ - lastNumFrames_;
                if(duration >= 90000*2 && num > 0 && inputBytes_ > 0){
                    fps_ = 90000* num / duration;
                    bitrate_ = 90000 * inputBytes_*8 /duration/1000;
                    lastNumFrames_ = numFrames_;
                    lastPTS4FPS_ = pts;
                    inputBytes_ = 0;
                }
                
                videoView_->drawYUV(decimg->d_w, decimg->d_h
                                            , decimg->planes[0], decimg->stride[0]
                                            , decimg->planes[1], decimg->stride[1]
                                            , decimg->planes[2], decimg->stride[2]);
                char txt[128];
                int txtlen = makeTextCommon(encodedLength, frameFlags, frameNo, txt+0);
                txtlen += sprintf(txt+txtlen, ", TL=%d", layerId );
                txtlen += sprintf(txt+txtlen, ", fps=%d", fps_ );
                txtlen += sprintf(txt+txtlen, ", br=%lld", bitrate_ );
                if(corrupted){
                    txtlen += sprintf(txt+txtlen, ",corrupted" );
                }
                txtlen += sprintf(txt+txtlen, ", upd(%s %s %s)"
                                  ,(refupdate & VP8_LAST_FRAME)?"LF":""
                                  ,(refupdate & VP8_GOLD_FRAME)?"GF":""
                                  ,(refupdate & VP8_ALTR_FRAME)?"ALF":"");
                txtlen += sprintf(txt+txtlen, ", ref(%s %s %s)"
                                  ,(refUsed & VP8_LAST_FRAME)?"LF":""
                                  ,(refUsed & VP8_GOLD_FRAME)?"GF":""
                                  ,(refUsed & VP8_ALTR_FRAME)?"ALF":"");
                
                frameTxtView_->draw(txt);
                odbgi("  decode=[%s]", txt);
            }
        }
    }
};


class AppDemo{
    //vpx_image_t * vpximg_ = NULL;
    
    EncodeUser * encodeUser_ = NULL;
    DecodeUser * decodeUser1_ = NULL;
    DecodeUser * decodeUser2_ = NULL;
    
    int numFrames_ = 0;
    size_t numEncodeBytes_ = 0;
    int framerate_ = 0;
    int keyInterval_ = 0;
    uint32_t startTime_ = 0;
    bool drop_ = false;
    vpx_codec_pts_t pts_ = 0;
    uint32_t duration_ = 90000 / 30;
public:
    virtual ~AppDemo(){
        this->close();
    }
    
    int start(){
        return 0;
    }
    
    void stop(){
        if(numFrames_ > 0){
            uint32_t elapsed = SDL_GetTicks() - startTime_;
            if(numFrames_ > 0 && elapsed > 0){
                odbgi("final: elapsed=%d ms, frames=%d, encBytes=%zu, fps=%d, bps=%zu"
                      , elapsed, numFrames_, numEncodeBytes_, 1000*numFrames_/elapsed, 8*numEncodeBytes_*framerate_/numFrames_);
            }
        }
        numFrames_ = 0;
        numEncodeBytes_ = 0;
        drop_ = false;
        pts_ = 0;
    }
    
    void close(){
        this->stop();
        if(encodeUser_){
            delete encodeUser_;
            encodeUser_ = NULL;
        }
        if(decodeUser1_){
            delete decodeUser1_;
            decodeUser1_ = NULL;
        }
        if(decodeUser2_){
            delete decodeUser2_;
            decodeUser2_ = NULL;
        }
    }
    
    int open(int width, int height, int framerate, int keyInterval = 0){
        int ret = -1;
        do{
            this->close();
            
            encodeUser_ = new EncodeUser("raw", width, height);
            decodeUser1_ = new DecodeUser("decode1", width, height, 1);
            decodeUser2_ = new DecodeUser("decode2", width, height, 0);
            
            
            ret = decodeUser1_->decoder_->open();
            if(ret) break;
            
            ret = decodeUser2_->decoder_->open();
            if(ret) break;
            
            int x1 = -1;
            int x2 = -1;
            int y = -1;
            SDL_DisplayMode displayMode;
            ret = SDL_GetCurrentDisplayMode(0, &displayMode);
            if(ret == 0){
                int cx = displayMode.w/2;
                int cy = displayMode.h/2;
                x1 = cx > width ? cx-width : 0;
                x2 = cx;
                y = cy > height ? cy-height/2 : 0;
            }

            encodeUser_->window_->open(x1, y);
            if(ret) break;

            decodeUser1_->window_->open(x2, y-height/2);
            if(ret) break;
            
            decodeUser2_->window_->open(x2, y+height/2);

            encodeUser_->frameTxtView_->draw(" press 'i' -> Key Frame, press 'p' -> P Frame, 'ctrl+' -> drop");
            decodeUser1_->frameTxtView_->draw(" press 'g' -> Play/Pause");
            decodeUser2_->frameTxtView_->draw(" press 'q' -> Quit");
            
            
            
            ret = this->start();
            if(ret) break;
            
            ret = 0;
        }while(0);
        if(ret){
            this->close();
        }
        return ret;
    }
    
    // bool isProcessing(){
    //     return fp_?true : false;
    // }
    
    int processOneFrame(vpx_image_t * vpximg, uint8_t * encodedFrame, int encodedLength
      , vpx_codec_frame_flags_t frameFlags, vpx_codec_pts_t pts, int layerId, bool skip=false){
        int ret = 0;
        do{
            if(numFrames_ == 0){
                startTime_ = SDL_GetTicks();
            }else{
            }

            odbgi("--- enc frame %d, pts=%lld", numFrames_, pts);
            encodeUser_->videoView_->drawYUV(vpximg->d_w, vpximg->d_h
                                 , vpximg->planes[0], vpximg->stride[0]
                                 , vpximg->planes[1], vpximg->stride[1]
                                 , vpximg->planes[2], vpximg->stride[2]);
            ++numFrames_;
            
            
            char txt[128];
            int txtlen = ::makeTextCommon(encodedLength, frameFlags, numFrames_, txt+0);
            if(skip){
                txtlen += sprintf(txt+txtlen, ", skip=%d", skip);
            }
            encodeUser_->frameTxtView_->draw(txt);

            numEncodeBytes_ += encodedLength;
            if(encodedLength > 0){
                dump_vp8_frame(encodedFrame, encodedLength);
                decodeUser1_->decode(pts, encodedFrame, encodedLength, frameFlags, numFrames_, layerId, skip);
                decodeUser2_->decode(pts, encodedFrame, encodedLength, frameFlags, numFrames_, layerId, skip);
            }
            
            ret = 0;
        }while(0);
        return ret;
    }
    
    void refresh(){
        if(encodeUser_){
            encodeUser_->window_->refresh();
        }
        if(decodeUser1_){
            decodeUser1_->window_->refresh();
        }
        if(decodeUser2_){
            decodeUser2_->window_->refresh();
        }
    }
};

// static
// int encode_file(const char * yuvfile, int width, int height, int framerate, int keyInterval = 0){
//     int ret = 0;
//     AppDemo * app = new AppDemo();
//     do{
//         ret = app->open(yuvfile, width, height, framerate, keyInterval);
//         if(ret) break;
        
//         int frameInterval = 1000 / framerate;
//         bool playing = false;
//         uint32_t nextDrawTime = SDL_GetTicks();
//         bool quitLoop = false;
//         SDL_Event event;
//         while(!quitLoop){
//             int timeout = 10;
//             uint32_t now_ms = SDL_GetTicks();
//             if(playing){
//                 if(now_ms >= nextDrawTime){
//                     ret = app->processOneFrame(false, false);
//                     if(ret) break;
//                     nextDrawTime += frameInterval;
//                     if(!app->isProcessing()){
//                         playing = false;
//                     }
//                 }
                
//                 now_ms = SDL_GetTicks();
//                 if(now_ms < nextDrawTime){
//                     timeout = (int)(nextDrawTime - now_ms);
//                 }
                
//             }
//             if(timeout > 0){
//                 SDL_WaitEventTimeout(NULL, timeout);
//             }
            
//             while (SDL_PollEvent(&event)) {
//                 if(event.type==SDL_QUIT){
//                     odbgi("got QUIT event %d", event.type);
//                     quitLoop = true;
//                     break;
//                 }else if(event.type == SDL_WINDOWEVENT){
//                     if(event.window.event == SDL_WINDOWEVENT_CLOSE){
//                         odbgi("Window %d closed", event.window.windowID);
//                         quitLoop = true;
//                         break;
//                     }else if(event.window.event == SDL_WINDOWEVENT_RESIZED){
//                         odbgi("Window %d resized to %dx%d"
//                               , event.window.windowID
//                               , event.window.data1, event.window.data2);
//                         app->refresh();
//                     }
//                 }else if(event.type == SDL_KEYDOWN){
//                     odbgi("got keydown event %d, win=%d, key=%d, scan=%d, mod=%d", event.type, event.key.windowID
//                           , event.key.keysym.sym, event.key.keysym.scancode, event.key.keysym.mod);
//                     if(event.key.keysym.sym == SDLK_p){
//                         bool drop = (event.key.keysym.mod & KMOD_CTRL)!=0;
//                         ret = app->processOneFrame(false, drop);
//                     }else if(event.key.keysym.sym == SDLK_i){
//                         bool drop = (event.key.keysym.mod & KMOD_CTRL)!=0;
//                         ret = app->processOneFrame(true, drop);
//                     }else if(event.key.keysym.sym == SDLK_g){
//                         playing = !playing;
//                         if(playing){
//                             app->start();
//                             nextDrawTime = SDL_GetTicks();
//                         }
//                     }else if(event.key.keysym.sym == SDLK_q){
//                         quitLoop = true;
//                     }
//                     if(ret){
//                         quitLoop = true;
//                     }
//                 }
//             }
            
            
//         }// while(!quitLoop)
//         ret = 0;
//     }while(0);
    
//     if(app){
//         delete app;
//         app = NULL;
//     }
//     return ret;
// }




#define ROI_MAP 0

#define zero(Dest) memset(&Dest, 0, sizeof(Dest));

static const char *exec_name;

void usage_exit(void) { exit(EXIT_FAILURE); }

// Denoiser states for vp8, for temporal denoising.
enum denoiserStateVp8 {
  kVp8DenoiserOff,
  kVp8DenoiserOnYOnly,
  kVp8DenoiserOnYUV,
  kVp8DenoiserOnYUVAggressive,
  kVp8DenoiserOnAdaptive
};

// Denoiser states for vp9, for temporal denoising.
enum denoiserStateVp9 {
  kVp9DenoiserOff,
  kVp9DenoiserOnYOnly,
  // For SVC: denoise the top two spatial layers.
  kVp9DenoiserOnYTwoSpatialLayers
};

static int mode_to_num_layers[13] = { 1, 2, 2, 3, 3, 3, 3, 5, 2, 3, 3, 3, 3 };

// For rate control encoding stats.
struct RateControlMetrics {
  // Number of input frames per layer.
  int layer_input_frames[VPX_TS_MAX_LAYERS];
  // Total (cumulative) number of encoded frames per layer.
  int layer_tot_enc_frames[VPX_TS_MAX_LAYERS];
  // Number of encoded non-key frames per layer.
  int layer_enc_frames[VPX_TS_MAX_LAYERS];
  // Framerate per layer layer (cumulative).
  double layer_framerate[VPX_TS_MAX_LAYERS];
  // Target average frame size per layer (per-frame-bandwidth per layer).
  double layer_pfb[VPX_TS_MAX_LAYERS];
  // Actual average frame size per layer.
  double layer_avg_frame_size[VPX_TS_MAX_LAYERS];
  // Average rate mismatch per layer (|target - actual| / target).
  double layer_avg_rate_mismatch[VPX_TS_MAX_LAYERS];
  // Actual encoding bitrate per layer (cumulative).
  double layer_encoding_bitrate[VPX_TS_MAX_LAYERS];
  // Average of the short-time encoder actual bitrate.
  // TODO(marpan): Should we add these short-time stats for each layer?
  double avg_st_encoding_bitrate;
  // Variance of the short-time encoder actual bitrate.
  double variance_st_encoding_bitrate;
  // Window (number of frames) for computing short-timee encoding bitrate.
  int window_size;
  // Number of window measurements.
  int window_count;
  int layer_target_bitrate[VPX_MAX_LAYERS];
};

// Note: these rate control metrics assume only 1 key frame in the
// sequence (i.e., first frame only). So for temporal pattern# 7
// (which has key frame for every frame on base layer), the metrics
// computation will be off/wrong.
// TODO(marpan): Update these metrics to account for multiple key frames
// in the stream.
static void set_rate_control_metrics(struct RateControlMetrics *rc,
                                     vpx_codec_enc_cfg_t *cfg) {
  unsigned int i = 0;
  // Set the layer (cumulative) framerate and the target layer (non-cumulative)
  // per-frame-bandwidth, for the rate control encoding stats below.
  const double framerate = cfg->g_timebase.den / cfg->g_timebase.num;
  rc->layer_framerate[0] = framerate / cfg->ts_rate_decimator[0];
  rc->layer_pfb[0] =
      1000.0 * rc->layer_target_bitrate[0] / rc->layer_framerate[0];
  for (i = 0; i < cfg->ts_number_layers; ++i) {
    if (i > 0) {
      rc->layer_framerate[i] = framerate / cfg->ts_rate_decimator[i];
      rc->layer_pfb[i] =
          1000.0 *
          (rc->layer_target_bitrate[i] - rc->layer_target_bitrate[i - 1]) /
          (rc->layer_framerate[i] - rc->layer_framerate[i - 1]);
    }
    rc->layer_input_frames[i] = 0;
    rc->layer_enc_frames[i] = 0;
    rc->layer_tot_enc_frames[i] = 0;
    rc->layer_encoding_bitrate[i] = 0.0;
    rc->layer_avg_frame_size[i] = 0.0;
    rc->layer_avg_rate_mismatch[i] = 0.0;
  }
  rc->window_count = 0;
  rc->window_size = 15;
  rc->avg_st_encoding_bitrate = 0.0;
  rc->variance_st_encoding_bitrate = 0.0;
}

static void printout_rate_control_summary(struct RateControlMetrics *rc,
                                          vpx_codec_enc_cfg_t *cfg,
                                          int frame_cnt) {
  unsigned int i = 0;
  int tot_num_frames = 0;
  double perc_fluctuation = 0.0;
  printf("Total number of processed frames: %d\n\n", frame_cnt - 1);
  printf("Rate control layer stats for %d layer(s):\n\n",
         cfg->ts_number_layers);
  for (i = 0; i < cfg->ts_number_layers; ++i) {
    const int num_dropped =
        (i > 0) ? (rc->layer_input_frames[i] - rc->layer_enc_frames[i])
                : (rc->layer_input_frames[i] - rc->layer_enc_frames[i] - 1);
    tot_num_frames += rc->layer_input_frames[i];
    rc->layer_encoding_bitrate[i] = 0.001 * rc->layer_framerate[i] *
                                    rc->layer_encoding_bitrate[i] /
                                    tot_num_frames;
    rc->layer_avg_frame_size[i] =
        rc->layer_avg_frame_size[i] / rc->layer_enc_frames[i];
    rc->layer_avg_rate_mismatch[i] =
        100.0 * rc->layer_avg_rate_mismatch[i] / rc->layer_enc_frames[i];
    printf("For layer#: %d \n", i);
    printf("Bitrate (target vs actual): %d %f \n", rc->layer_target_bitrate[i],
           rc->layer_encoding_bitrate[i]);
    printf("Average frame size (target vs actual): %f %f \n", rc->layer_pfb[i],
           rc->layer_avg_frame_size[i]);
    printf("Average rate_mismatch: %f \n", rc->layer_avg_rate_mismatch[i]);
    printf(
        "Number of input frames, encoded (non-key) frames, "
        "and perc dropped frames: %d %d %f \n",
        rc->layer_input_frames[i], rc->layer_enc_frames[i],
        100.0 * num_dropped / rc->layer_input_frames[i]);
    printf("\n");
  }
  rc->avg_st_encoding_bitrate = rc->avg_st_encoding_bitrate / rc->window_count;
  rc->variance_st_encoding_bitrate =
      rc->variance_st_encoding_bitrate / rc->window_count -
      (rc->avg_st_encoding_bitrate * rc->avg_st_encoding_bitrate);
  perc_fluctuation = 100.0 * sqrt(rc->variance_st_encoding_bitrate) /
                     rc->avg_st_encoding_bitrate;
  printf("Short-time stats, for window of %d frames: \n", rc->window_size);
  printf("Average, rms-variance, and percent-fluct: %f %f %f \n",
         rc->avg_st_encoding_bitrate, sqrt(rc->variance_st_encoding_bitrate),
         perc_fluctuation);
  if ((frame_cnt - 1) != tot_num_frames)
    die("Error: Number of input frames not equal to output! \n");
}

#if ROI_MAP
static void set_roi_map(const char *enc_name, vpx_codec_enc_cfg_t *cfg,
                        vpx_roi_map_t *roi) {
  unsigned int i, j;
  int block_size = 0;
  uint8_t is_vp8 = strncmp(enc_name, "vp8", 3) == 0 ? 1 : 0;
  uint8_t is_vp9 = strncmp(enc_name, "vp9", 3) == 0 ? 1 : 0;
  if (!is_vp8 && !is_vp9) {
    die("unsupported codec.");
  }
  zero(*roi);

  block_size = is_vp9 && !is_vp8 ? 8 : 16;

  // ROI is based on the segments (4 for vp8, 8 for vp9), smallest unit for
  // segment is 16x16 for vp8, 8x8 for vp9.
  roi->rows = (cfg->g_h + block_size - 1) / block_size;
  roi->cols = (cfg->g_w + block_size - 1) / block_size;

  // Applies delta QP on the segment blocks, varies from -63 to 63.
  // Setting to negative means lower QP (better quality).
  // Below we set delta_q to the extreme (-63) to show strong effect.
  // VP8 uses the first 4 segments. VP9 uses all 8 segments.
  zero(roi->delta_q);
  roi->delta_q[1] = -63;

  // Applies delta loopfilter strength on the segment blocks, varies from -63 to
  // 63. Setting to positive means stronger loopfilter. VP8 uses the first 4
  // segments. VP9 uses all 8 segments.
  zero(roi->delta_lf);

  if (is_vp8) {
    // Applies skip encoding threshold on the segment blocks, varies from 0 to
    // UINT_MAX. Larger value means more skipping of encoding is possible.
    // This skip threshold only applies on delta frames.
    zero(roi->static_threshold);
  }

  if (is_vp9) {
    // Apply skip segment. Setting to 1 means this block will be copied from
    // previous frame.
    zero(roi->skip);
  }

  if (is_vp9) {
    // Apply ref frame segment.
    // -1 : Do not apply this segment.
    //  0 : Froce using intra.
    //  1 : Force using last.
    //  2 : Force using golden.
    //  3 : Force using alfref but not used in non-rd pickmode for 0 lag.
    memset(roi->ref_frame, -1, sizeof(roi->ref_frame));
    roi->ref_frame[1] = 1;
  }

  // Use 2 states: 1 is center square, 0 is the rest.
  roi->roi_map =
      (uint8_t *)calloc(roi->rows * roi->cols, sizeof(*roi->roi_map));
  for (i = 0; i < roi->rows; ++i) {
    for (j = 0; j < roi->cols; ++j) {
      if (i > (roi->rows >> 2) && i < ((roi->rows * 3) >> 2) &&
          j > (roi->cols >> 2) && j < ((roi->cols * 3) >> 2)) {
        roi->roi_map[i * roi->cols + j] = 1;
      }
    }
  }
}
#endif

// Temporal scaling parameters:
// NOTE: The 3 prediction frames cannot be used interchangeably due to
// differences in the way they are handled throughout the code. The
// frames should be allocated to layers in the order LAST, GF, ARF.
// Other combinations work, but may produce slightly inferior results.
static void set_temporal_layer_pattern(int layering_mode,
                                       vpx_codec_enc_cfg_t *cfg,
                                       int *layer_flags,
                                       int *flag_periodicity) {
  odbgi("layering_mode = %d", layering_mode);
  switch (layering_mode) {
    case 0: {
      // 1-layer.
      int ids[1] = { 0 };
      cfg->ts_periodicity = 1;
      *flag_periodicity = 1;
      cfg->ts_number_layers = 1;
      cfg->ts_rate_decimator[0] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // Update L only.
      layer_flags[0] =
          VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF;
      break;
    }
    case 1: {
      // 2-layers, 2-frame period.
      int ids[2] = { 0, 1 };
      cfg->ts_periodicity = 2;
      *flag_periodicity = 2;
      cfg->ts_number_layers = 2;
      cfg->ts_rate_decimator[0] = 2;
      cfg->ts_rate_decimator[1] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
#if 1
      // 0=L, 1=GF, Intra-layer prediction enabled.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF;
      layer_flags[1] =
          VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_REF_ARF;
#else
      // 0=L, 1=GF, Intra-layer prediction disabled.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF;
      layer_flags[1] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST |
                       VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_REF_LAST;
#endif
      break;
    }
    case 2: {
      // 2-layers, 3-frame period.
      int ids[3] = { 0, 1, 1 };
      cfg->ts_periodicity = 3;
      *flag_periodicity = 3;
      cfg->ts_number_layers = 2;
      cfg->ts_rate_decimator[0] = 3;
      cfg->ts_rate_decimator[1] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, Intra-layer prediction enabled.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF;
      layer_flags[1] = layer_flags[2] =
          VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_ARF |
          VP8_EFLAG_NO_UPD_LAST;
      break;
    }
    case 3: {
      // 3-layers, 6-frame period.
      int ids[6] = { 0, 2, 2, 1, 2, 2 };
      cfg->ts_periodicity = 6;
      *flag_periodicity = 6;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 6;
      cfg->ts_rate_decimator[1] = 3;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF, Intra-layer prediction enabled.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF;
      layer_flags[3] =
          VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST;
      layer_flags[1] = layer_flags[2] = layer_flags[4] = layer_flags[5] =
          VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_LAST;
      break;
    }
    case 4: {
      // 3-layers, 4-frame period.
      int ids[4] = { 0, 2, 1, 2 };
      cfg->ts_periodicity = 4;
      *flag_periodicity = 4;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 4;
      cfg->ts_rate_decimator[1] = 2;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF, Intra-layer prediction disabled.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF;
      layer_flags[2] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST;
      layer_flags[1] = layer_flags[3] =
          VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF |
          VP8_EFLAG_NO_UPD_ARF;
      break;
    }
    case 5: {
      // 3-layers, 4-frame period.
      int ids[4] = { 0, 2, 1, 2 };
      cfg->ts_periodicity = 4;
      *flag_periodicity = 4;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 4;
      cfg->ts_rate_decimator[1] = 2;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF, Intra-layer prediction enabled in layer 1, disabled
      // in layer 2.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF;
      layer_flags[2] =
          VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ARF;
      layer_flags[1] = layer_flags[3] =
          VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF |
          VP8_EFLAG_NO_UPD_ARF;
      break;
    }
    case 6: {
      // 3-layers, 4-frame period.
      int ids[4] = { 0, 2, 1, 2 };
      cfg->ts_periodicity = 4;
      *flag_periodicity = 4;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 4;
      cfg->ts_rate_decimator[1] = 2;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF, Intra-layer prediction enabled.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF;
      layer_flags[2] =
          VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ARF;
      layer_flags[1] = layer_flags[3] =
          VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF;
      break;
    }
    case 7: {
      // NOTE: Probably of academic interest only.
      // 5-layers, 16-frame period.
      int ids[16] = { 0, 4, 3, 4, 2, 4, 3, 4, 1, 4, 3, 4, 2, 4, 3, 4 };
      cfg->ts_periodicity = 16;
      *flag_periodicity = 16;
      cfg->ts_number_layers = 5;
      cfg->ts_rate_decimator[0] = 16;
      cfg->ts_rate_decimator[1] = 8;
      cfg->ts_rate_decimator[2] = 4;
      cfg->ts_rate_decimator[3] = 2;
      cfg->ts_rate_decimator[4] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      layer_flags[0] = VPX_EFLAG_FORCE_KF;
      layer_flags[1] = layer_flags[3] = layer_flags[5] = layer_flags[7] =
          layer_flags[9] = layer_flags[11] = layer_flags[13] = layer_flags[15] =
              VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF |
              VP8_EFLAG_NO_UPD_ARF;
      layer_flags[2] = layer_flags[6] = layer_flags[10] = layer_flags[14] =
          VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_GF;
      layer_flags[4] = layer_flags[12] =
          VP8_EFLAG_NO_REF_LAST | VP8_EFLAG_NO_UPD_ARF;
      layer_flags[8] = VP8_EFLAG_NO_REF_LAST | VP8_EFLAG_NO_REF_GF;
      break;
    }
    case 8: {
      // 2-layers, with sync point at first frame of layer 1.
      int ids[2] = { 0, 1 };
      cfg->ts_periodicity = 2;
      *flag_periodicity = 8;
      cfg->ts_number_layers = 2;
      cfg->ts_rate_decimator[0] = 2;
      cfg->ts_rate_decimator[1] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF.
      // ARF is used as predictor for all frames, and is only updated on
      // key frame. Sync point every 8 frames.

      // Layer 0: predict from L and ARF, update L and G.
      layer_flags[0] =
          VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_ARF;
      // Layer 1: sync point: predict from L and ARF, and update G.
      layer_flags[1] =
          VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ARF;
      // Layer 0, predict from L and ARF, update L.
      layer_flags[2] =
          VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF;
      // Layer 1: predict from L, G and ARF, and update G.
      layer_flags[3] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST |
                       VP8_EFLAG_NO_UPD_ENTROPY;
      // Layer 0.
      layer_flags[4] = layer_flags[2];
      // Layer 1.
      layer_flags[5] = layer_flags[3];
      // Layer 0.
      layer_flags[6] = layer_flags[4];
      // Layer 1.
      layer_flags[7] = layer_flags[5];
      break;
    }
    case 9: {
      // 3-layers: Sync points for layer 1 and 2 every 8 frames.
      int ids[4] = { 0, 2, 1, 2 };
      cfg->ts_periodicity = 4;
      *flag_periodicity = 8;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 4;
      cfg->ts_rate_decimator[1] = 2;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF.
      layer_flags[0] = VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_REF_GF |
                       VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF;
      layer_flags[1] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF;
      layer_flags[2] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ARF;
      layer_flags[3] = layer_flags[5] =
          VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF;
      layer_flags[4] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF;
      layer_flags[6] =
          VP8_EFLAG_NO_REF_ARF | VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ARF;
      layer_flags[7] = VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_ENTROPY;
      break;
    }
    case 10: {
      // 3-layers structure where ARF is used as predictor for all frames,
      // and is only updated on key frame.
      // Sync points for layer 1 and 2 every 8 frames.

      int ids[4] = { 0, 2, 1, 2 };
      cfg->ts_periodicity = 4;
      *flag_periodicity = 8;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 4;
      cfg->ts_rate_decimator[1] = 2;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF.
      // Layer 0: predict from L and ARF; update L and G.
      layer_flags[0] =
          VPX_EFLAG_FORCE_KF | VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF;
      // Layer 2: sync point: predict from L and ARF; update none.
      layer_flags[1] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_GF |
                       VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST |
                       VP8_EFLAG_NO_UPD_ENTROPY;
      // Layer 1: sync point: predict from L and ARF; update G.
      layer_flags[2] =
          VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST;
      // Layer 2: predict from L, G, ARF; update none.
      layer_flags[3] = VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF |
                       VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ENTROPY;
      // Layer 0: predict from L and ARF; update L.
      layer_flags[4] =
          VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF;
      // Layer 2: predict from L, G, ARF; update none.
      layer_flags[5] = layer_flags[3];
      // Layer 1: predict from L, G, ARF; update G.
      layer_flags[6] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST;
      // Layer 2: predict from L, G, ARF; update none.
      layer_flags[7] = layer_flags[3];
      break;
    }
    case 11: {
      // 3-layers structure with one reference frame.
      // This works same as temporal_layering_mode 3.
      // This was added to compare with vp9_spatial_svc_encoder.

      // 3-layers, 4-frame period.
      int ids[4] = { 0, 2, 1, 2 };
      cfg->ts_periodicity = 4;
      *flag_periodicity = 4;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 4;
      cfg->ts_rate_decimator[1] = 2;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF, Intra-layer prediction disabled.
      layer_flags[0] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF;
      layer_flags[2] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST;
      layer_flags[1] = VP8_EFLAG_NO_REF_GF | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF;
      layer_flags[3] = VP8_EFLAG_NO_REF_LAST | VP8_EFLAG_NO_REF_ARF |
                       VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_GF;
      break;
    }
    case 12:
    default: {
      // 3-layers structure as in case 10, but no sync/refresh points for
      // layer 1 and 2.
      int ids[4] = { 0, 2, 1, 2 };
      cfg->ts_periodicity = 4;
      *flag_periodicity = 8;
      cfg->ts_number_layers = 3;
      cfg->ts_rate_decimator[0] = 4;
      cfg->ts_rate_decimator[1] = 2;
      cfg->ts_rate_decimator[2] = 1;
      memcpy(cfg->ts_layer_id, ids, sizeof(ids));
      // 0=L, 1=GF, 2=ARF.
      // Layer 0: predict from L and ARF; update L.
      layer_flags[0] =
          VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_REF_GF;
      layer_flags[4] = layer_flags[0];
      // Layer 1: predict from L, G, ARF; update G.
      layer_flags[2] = VP8_EFLAG_NO_UPD_ARF | VP8_EFLAG_NO_UPD_LAST;
      layer_flags[6] = layer_flags[2];
      // Layer 2: predict from L, G, ARF; update none.
      layer_flags[1] = VP8_EFLAG_NO_UPD_GF | VP8_EFLAG_NO_UPD_ARF |
                       VP8_EFLAG_NO_UPD_LAST | VP8_EFLAG_NO_UPD_ENTROPY;
      layer_flags[3] = layer_flags[1];
      layer_flags[5] = layer_flags[1];
      layer_flags[7] = layer_flags[1];
      break;
    }
  }
}

int main(int argc, char **argv) {
  VpxVideoWriter *outfile[VPX_TS_MAX_LAYERS] = { NULL };
  vpx_codec_ctx_t codec;
  vpx_codec_enc_cfg_t cfg;
  int frame_cnt = 0;
  vpx_image_t raw;
  vpx_codec_err_t res;
  unsigned int width;
  unsigned int height;
  uint32_t error_resilient = 0;
  int speed;
  int frame_avail;
  int got_data;
  int flags = 0;
  unsigned int i;
  int pts = 0;             // PTS starts at 0.
  int frame_duration = 1;  // 1 timebase tick per frame.
  int layering_mode = 0;
  int layer_flags[VPX_TS_MAX_PERIODICITY] = { 0 };
  int flag_periodicity = 1;
#if ROI_MAP
  vpx_roi_map_t roi;
#endif
  vpx_svc_layer_id_t layer_id = { 0, 0 };
  const VpxInterface *encoder = NULL;
  FILE *infile = NULL;
  struct RateControlMetrics rc;
  int64_t cx_time = 0;
  const int min_args_base = 13;
#if CONFIG_VP9_HIGHBITDEPTH
  vpx_bit_depth_t bit_depth = VPX_BITS_8;
  int input_bit_depth = 8;
  const int min_args = min_args_base + 1;
#else
  const int min_args = min_args_base;
#endif  // CONFIG_VP9_HIGHBITDEPTH
  double sum_bitrate = 0.0;
  double sum_bitrate2 = 0.0;
  double framerate = 30.0;

  zero(rc.layer_target_bitrate);

  exec_name = argv[0];
  // Check usage and arguments.
  if (argc < min_args) {
#if CONFIG_VP9_HIGHBITDEPTH
    die("Usage: %s <infile> <outfile> <codec_type(vp8/vp9)> <width> <height> "
        "<rate_num> <rate_den> <speed> <frame_drop_threshold> "
        "<error_resilient> <threads> <mode> "
        "<Rate_0> ... <Rate_nlayers-1> <bit-depth> \n",
        argv[0]);
#else
    die("Usage: %s <infile> <outfile> <codec_type(vp8/vp9)> <width> <height> "
        "<rate_num> <rate_den> <speed> <frame_drop_threshold> "
        "<error_resilient> <threads> <mode> "
        "<Rate_0> ... <Rate_nlayers-1> \n",
        argv[0]);
#endif  // CONFIG_VP9_HIGHBITDEPTH
  }

  encoder = get_vpx_encoder_by_name(argv[3]);
  if (!encoder) die("Unsupported codec.");

  printf("Using %s\n", vpx_codec_iface_name(encoder->codec_interface()));

  width = (unsigned int)strtoul(argv[4], NULL, 0);
  height = (unsigned int)strtoul(argv[5], NULL, 0);
  if (width < 16 || width % 2 || height < 16 || height % 2) {
    die("Invalid resolution: %d x %d", width, height);
  }

  layering_mode = (int)strtol(argv[12], NULL, 0);
  if (layering_mode < 0 || layering_mode > 13) {
    die("Invalid layering mode (0..12) %s", argv[12]);
  }

  if (argc != min_args + mode_to_num_layers[layering_mode]) {
    die("Invalid number of arguments");
  }

#if CONFIG_VP9_HIGHBITDEPTH
  switch (strtol(argv[argc - 1], NULL, 0)) {
    case 8:
      bit_depth = VPX_BITS_8;
      input_bit_depth = 8;
      break;
    case 10:
      bit_depth = VPX_BITS_10;
      input_bit_depth = 10;
      break;
    case 12:
      bit_depth = VPX_BITS_12;
      input_bit_depth = 12;
      break;
    default: die("Invalid bit depth (8, 10, 12) %s", argv[argc - 1]);
  }
  if (!vpx_img_alloc(
          &raw, bit_depth == VPX_BITS_8 ? VPX_IMG_FMT_I420 : VPX_IMG_FMT_I42016,
          width, height, 32)) {
    die("Failed to allocate image", width, height);
  }
#else
  if (!vpx_img_alloc(&raw, VPX_IMG_FMT_I420, width, height, 32)) {
    die("Failed to allocate image", width, height);
  }
#endif  // CONFIG_VP9_HIGHBITDEPTH

  // Populate encoder configuration.
  res = vpx_codec_enc_config_default(encoder->codec_interface(), &cfg, 0);
  if (res) {
    printf("Failed to get config: %s\n", vpx_codec_err_to_string(res));
    return EXIT_FAILURE;
  }

  // Update the default configuration with our settings.
  cfg.g_w = width;
  cfg.g_h = height;

#if CONFIG_VP9_HIGHBITDEPTH
  if (bit_depth != VPX_BITS_8) {
    cfg.g_bit_depth = bit_depth;
    cfg.g_input_bit_depth = input_bit_depth;
    cfg.g_profile = 2;
  }
#endif  // CONFIG_VP9_HIGHBITDEPTH

  // Timebase format e.g. 30fps: numerator=1, demoninator = 30.
  cfg.g_timebase.num = (int)strtol(argv[6], NULL, 0);
  cfg.g_timebase.den = (int)strtol(argv[7], NULL, 0);

  speed = (int)strtol(argv[8], NULL, 0);
  if (speed < 0) {
    die("Invalid speed setting: must be positive");
  }

  for (i = min_args_base;
       (int)i < min_args_base + mode_to_num_layers[layering_mode]; ++i) {
    rc.layer_target_bitrate[i - 13] = (int)strtol(argv[i], NULL, 0);
    if (strncmp(encoder->name, "vp8", 3) == 0)
      cfg.ts_target_bitrate[i - 13] = rc.layer_target_bitrate[i - 13];
    else if (strncmp(encoder->name, "vp9", 3) == 0)
      cfg.layer_target_bitrate[i - 13] = rc.layer_target_bitrate[i - 13];
  }

  // Real time parameters.
  cfg.rc_dropframe_thresh = (unsigned int)strtoul(argv[9], NULL, 0);
  cfg.rc_end_usage = VPX_CBR;
  cfg.rc_min_quantizer = 2;
  cfg.rc_max_quantizer = 56;
  if (strncmp(encoder->name, "vp9", 3) == 0) cfg.rc_max_quantizer = 52;
  cfg.rc_undershoot_pct = 50;
  cfg.rc_overshoot_pct = 50;
  cfg.rc_buf_initial_sz = 600;
  cfg.rc_buf_optimal_sz = 600;
  cfg.rc_buf_sz = 1000;

  // Disable dynamic resizing by default.
  cfg.rc_resize_allowed = 0;

  // Use 1 thread as default.
  cfg.g_threads = (unsigned int)strtoul(argv[11], NULL, 0);

  error_resilient = (uint32_t)strtoul(argv[10], NULL, 0);
  if (error_resilient != 0 && error_resilient != 1) {
    die("Invalid value for error resilient (0, 1): %d.", error_resilient);
  }
  // Enable error resilient mode.
  cfg.g_error_resilient = error_resilient;
  cfg.g_lag_in_frames = 0;
  cfg.kf_mode = VPX_KF_AUTO;

  // Disable automatic keyframe placement.
  cfg.kf_min_dist = cfg.kf_max_dist = 3000;

  cfg.temporal_layering_mode = VP9E_TEMPORAL_LAYERING_MODE_BYPASS;

  set_temporal_layer_pattern(layering_mode, &cfg, layer_flags,
                             &flag_periodicity);

  set_rate_control_metrics(&rc, &cfg);

  // Target bandwidth for the whole stream.
  // Set to layer_target_bitrate for highest layer (total bitrate).
  cfg.rc_target_bitrate = rc.layer_target_bitrate[cfg.ts_number_layers - 1];

  // Open input file.
  if (!(infile = fopen(argv[1], "rb"))) {
    die("Failed to open %s for reading", argv[1]);
  }

  framerate = cfg.g_timebase.den / cfg.g_timebase.num;
  // Open an output file for each stream.
  for (i = 0; i < cfg.ts_number_layers; ++i) {
    char file_name[PATH_MAX];
    VpxVideoInfo info;
    info.codec_fourcc = encoder->fourcc;
    info.frame_width = cfg.g_w;
    info.frame_height = cfg.g_h;
    info.time_base.numerator = cfg.g_timebase.num;
    info.time_base.denominator = cfg.g_timebase.den;

    snprintf(file_name, sizeof(file_name), "%s_%d.ivf", argv[2], i);
    outfile[i] = vpx_video_writer_open(file_name, kContainerIVF, &info);
    if (!outfile[i]) die("Failed to open %s for writing", file_name);

    assert(outfile[i] != NULL);
  }
  // No spatial layers in this encoder.
  cfg.ss_number_layers = 1;

// Initialize codec.
#if CONFIG_VP9_HIGHBITDEPTH
  if (vpx_codec_enc_init(
          &codec, encoder->codec_interface(), &cfg,
          bit_depth == VPX_BITS_8 ? 0 : VPX_CODEC_USE_HIGHBITDEPTH))
#else
  if (vpx_codec_enc_init(&codec, encoder->codec_interface(), &cfg, 0))
#endif  // CONFIG_VP9_HIGHBITDEPTH
    die_codec(&codec, "Failed to initialize encoder");

  if (strncmp(encoder->name, "vp8", 3) == 0) {
    vpx_codec_control(&codec, VP8E_SET_CPUUSED, -speed);
    vpx_codec_control(&codec, VP8E_SET_NOISE_SENSITIVITY, kVp8DenoiserOff);
    vpx_codec_control(&codec, VP8E_SET_STATIC_THRESHOLD, 1);
    vpx_codec_control(&codec, VP8E_SET_GF_CBR_BOOST_PCT, 0);
#if ROI_MAP
    set_roi_map(encoder->name, &cfg, &roi);
    if (vpx_codec_control(&codec, VP8E_SET_ROI_MAP, &roi))
      die_codec(&codec, "Failed to set ROI map");
#endif

  } else if (strncmp(encoder->name, "vp9", 3) == 0) {
    vpx_svc_extra_cfg_t svc_params;
    memset(&svc_params, 0, sizeof(svc_params));
    vpx_codec_control(&codec, VP8E_SET_CPUUSED, speed);
    vpx_codec_control(&codec, VP9E_SET_AQ_MODE, 3);
    vpx_codec_control(&codec, VP9E_SET_GF_CBR_BOOST_PCT, 0);
    vpx_codec_control(&codec, VP9E_SET_FRAME_PARALLEL_DECODING, 0);
    vpx_codec_control(&codec, VP9E_SET_FRAME_PERIODIC_BOOST, 0);
    vpx_codec_control(&codec, VP9E_SET_NOISE_SENSITIVITY, kVp9DenoiserOff);
    vpx_codec_control(&codec, VP8E_SET_STATIC_THRESHOLD, 1);
    vpx_codec_control(&codec, VP9E_SET_TUNE_CONTENT, 0);
    vpx_codec_control(&codec, VP9E_SET_TILE_COLUMNS, get_msb(cfg.g_threads));
#if ROI_MAP
    set_roi_map(encoder->name, &cfg, &roi);
    if (vpx_codec_control(&codec, VP9E_SET_ROI_MAP, &roi))
      die_codec(&codec, "Failed to set ROI map");
    vpx_codec_control(&codec, VP9E_SET_AQ_MODE, 0);
#endif
    // TODO(marpan/jianj): There is an issue with row-mt for low resolutons at
    // high speed settings, disable its use for those cases for now.
    if (cfg.g_threads > 1 && ((cfg.g_w > 320 && cfg.g_h > 240) || speed < 7))
      vpx_codec_control(&codec, VP9E_SET_ROW_MT, 1);
    else
      vpx_codec_control(&codec, VP9E_SET_ROW_MT, 0);
    if (vpx_codec_control(&codec, VP9E_SET_SVC, layering_mode > 0 ? 1 : 0))
      die_codec(&codec, "Failed to set SVC");
    for (i = 0; i < cfg.ts_number_layers; ++i) {
      svc_params.max_quantizers[i] = cfg.rc_max_quantizer;
      svc_params.min_quantizers[i] = cfg.rc_min_quantizer;
    }
    svc_params.scaling_factor_num[0] = cfg.g_h;
    svc_params.scaling_factor_den[0] = cfg.g_h;
    vpx_codec_control(&codec, VP9E_SET_SVC_PARAMETERS, &svc_params);
  }
  if (strncmp(encoder->name, "vp8", 3) == 0) {
    vpx_codec_control(&codec, VP8E_SET_SCREEN_CONTENT_MODE, 0);
  }
  vpx_codec_control(&codec, VP8E_SET_TOKEN_PARTITIONS, 1);
  // This controls the maximum target size of the key frame.
  // For generating smaller key frames, use a smaller max_intra_size_pct
  // value, like 100 or 200.
  {
    const int max_intra_size_pct = 1000;
    vpx_codec_control(&codec, VP8E_SET_MAX_INTRA_BITRATE_PCT,
                      max_intra_size_pct);
  }

    int ret = SDL_Init(SDL_INIT_VIDEO) ;
    if(ret){
        odbge( "Could not initialize SDL - %s\n", SDL_GetError());
        return -1;
    }
    ret = TTF_Init();

    AppDemo * app = new AppDemo();
    app->open(width, height, 25);
    uint32_t duration = 90000 / 25;

    //frame_duration = duration;
    //duration = 1;


  frame_avail = 1;
  while (frame_avail || got_data) {
    struct vpx_usec_timer timer;
    vpx_codec_iter_t iter = NULL;
    const vpx_codec_cx_pkt_t *pkt;
    // Update the temporal layer_id. No spatial layers in this test.
    layer_id.spatial_layer_id = 0;
    layer_id.temporal_layer_id =
        cfg.ts_layer_id[frame_cnt % cfg.ts_periodicity];
    if (strncmp(encoder->name, "vp9", 3) == 0) {
      vpx_codec_control(&codec, VP9E_SET_SVC_LAYER_ID, &layer_id);
    } else if (strncmp(encoder->name, "vp8", 3) == 0) {
      vpx_codec_control(&codec, VP8E_SET_TEMPORAL_LAYER_ID,
                        layer_id.temporal_layer_id);
    }
    flags = layer_flags[frame_cnt % flag_periodicity];
    if (layering_mode == 0) flags = 0;
    frame_avail = vpx_img_read(&raw, infile);
    if (frame_avail) ++rc.layer_input_frames[layer_id.temporal_layer_id];
    vpx_usec_timer_start(&timer);
    if (vpx_codec_encode(&codec, frame_avail ? &raw : NULL, pts, 1, flags,
                         VPX_DL_REALTIME)) {
      die_codec(&codec, "Failed to encode frame");
    }
    vpx_usec_timer_mark(&timer);
    cx_time += vpx_usec_timer_elapsed(&timer);
    // Reset KF flag.
    if (layering_mode != 7) {
      layer_flags[0] &= ~VPX_EFLAG_FORCE_KF;
    }
    got_data = 0;
    while ((pkt = vpx_codec_get_cx_data(&codec, &iter))) {
      got_data = 1;
      switch (pkt->kind) {
        case VPX_CODEC_CX_FRAME_PKT:
          i = cfg.ts_layer_id[frame_cnt % cfg.ts_periodicity];
          app->processOneFrame(&raw, (uint8_t *)pkt->data.frame.buf, pkt->data.frame.sz, pkt->data.frame.flags, pts*duration, i);

          for (i = cfg.ts_layer_id[frame_cnt % cfg.ts_periodicity];
               i < cfg.ts_number_layers; ++i) {
            vpx_video_writer_write_frame(outfile[i], (const uint8_t *)pkt->data.frame.buf,
                                         pkt->data.frame.sz, pts);
            ++rc.layer_tot_enc_frames[i];
            rc.layer_encoding_bitrate[i] += 8.0 * pkt->data.frame.sz;
            // Keep count of rate control stats per layer (for non-key frames).
            if (i == cfg.ts_layer_id[frame_cnt % cfg.ts_periodicity] &&
                !(pkt->data.frame.flags & VPX_FRAME_IS_KEY)) {
              rc.layer_avg_frame_size[i] += 8.0 * pkt->data.frame.sz;
              rc.layer_avg_rate_mismatch[i] +=
                  fabs(8.0 * pkt->data.frame.sz - rc.layer_pfb[i]) /
                  rc.layer_pfb[i];
              ++rc.layer_enc_frames[i];
            }
          }
          // Update for short-time encoding bitrate states, for moving window
          // of size rc->window, shifted by rc->window / 2.
          // Ignore first window segment, due to key frame.
          if (frame_cnt > rc.window_size) {
            sum_bitrate += 0.001 * 8.0 * pkt->data.frame.sz * framerate;
            if (frame_cnt % rc.window_size == 0) {
              rc.window_count += 1;
              rc.avg_st_encoding_bitrate += sum_bitrate / rc.window_size;
              rc.variance_st_encoding_bitrate +=
                  (sum_bitrate / rc.window_size) *
                  (sum_bitrate / rc.window_size);
              sum_bitrate = 0.0;
            }
          }
          // Second shifted window.
          if (frame_cnt > rc.window_size + rc.window_size / 2) {
            sum_bitrate2 += 0.001 * 8.0 * pkt->data.frame.sz * framerate;
            if (frame_cnt > 2 * rc.window_size &&
                frame_cnt % rc.window_size == 0) {
              rc.window_count += 1;
              rc.avg_st_encoding_bitrate += sum_bitrate2 / rc.window_size;
              rc.variance_st_encoding_bitrate +=
                  (sum_bitrate2 / rc.window_size) *
                  (sum_bitrate2 / rc.window_size);
              sum_bitrate2 = 0.0;
            }
          }
          break;
        default: break;
      }
    }
    ++frame_cnt;
    pts += frame_duration;
    // if(frame_cnt >= 40){
    //   odbgi("reach max frames");
    //   break;
    // }
            bool quitLoop = false;
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if(event.type==SDL_QUIT){
                    odbgi("got QUIT event %d", event.type);
                    quitLoop = true;
                    break;
                }else if(event.type == SDL_WINDOWEVENT){
                    if(event.window.event == SDL_WINDOWEVENT_CLOSE){
                        odbgi("Window %d closed", event.window.windowID);
                        quitLoop = true;
                        break;
                    }else if(event.window.event == SDL_WINDOWEVENT_RESIZED){
                        odbgi("Window %d resized to %dx%d"
                              , event.window.windowID
                              , event.window.data1, event.window.data2);
                        app->refresh();
                    }
                }
            }
            if(quitLoop){
              break;
            }

  }
  fclose(infile);
  printout_rate_control_summary(&rc, &cfg, frame_cnt);
  printf("\n");
  printf("Frame cnt and encoding time/FPS stats for encoding: %d %f %f \n",
         frame_cnt, 1000 * (float)cx_time / (double)(frame_cnt * 1000000),
         1000000 * (double)frame_cnt / (double)cx_time);

  if (vpx_codec_destroy(&codec)) die_codec(&codec, "Failed to destroy codec");

  // Try to rewrite the output file headers with the actual frame count.
  for (i = 0; i < cfg.ts_number_layers; ++i) vpx_video_writer_close(outfile[i]);

  vpx_img_free(&raw);
#if ROI_MAP
  free(roi.roi_map);
#endif
  return EXIT_SUCCESS;
}
