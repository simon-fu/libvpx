
infile="/Users/simon/Desktop/simon/projects/easemob/src/xmedia/LearningRTC/data/test_yuv420p_640x360.yuv"
outfile="./out_svc"
codec_type="vp8" # vp8/vp9
width="640"
height="360"
rate_num="1"
rate_den="25"
speed="0"
frame_drop_threshold="30"
error_resilient="1"
threads="1"
mode="2" # layering_mode, reference mode_to_num_layers
rate0="360"
rate1="600"
rate2="1000"
cmdline="./bin_vpx_temporal_svc_encoder_gui"\
" $infile"\
" $outfile"\
" $codec_type"\
" $width"\
" $height"\
" $rate_num"\
" $rate_den"\
" $speed"\
" $frame_drop_threshold"\
" $error_resilient"\
" $threads"\
" $mode"\
" $rate0"\
" $rate1"\
""

echo cmdline=$cmdline
$cmdline

# ./vpx_temporal_svc_encoder <infile> <outfile> <codec_type(vp8/vp9)> <width> <height> <rate_num> <rate_den> <speed> <frame_drop_threshold> <error_resilient> <threads> <mode> <Rate_0> ... <Rate_nlayers-1>
