
g++ \
../tools_common.c \
../video_writer.c \
../ivfenc.c \
vpx_temporal_svc_encoder_gui.cc \
-std=c++11 \
-lvpx \
-lSDL2 \
-lSDL2_ttf \
-I ../out/ \
-I ../ \
-I /Users/simon/Desktop/simon/projects/easemob/src/xmedia/rtc-lab/src \
-L ../out/ \
-o bin_vpx_temporal_svc_encoder_gui

