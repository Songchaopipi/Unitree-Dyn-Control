## ==== walk-wbc标准版 ====

1. wbc_bruce_weighted+ALIP-deadbeat移植自Romoco中的实现和Bruce官方WBC, 在此基础上添加了简单的IK

## ==== stand-srbd-mpc+IK====
./build/g1_srbd_ik_stand --duration 12.1 --print --print-hz 1.0

./build/g1_srbd_ik_walk_rp --duration 14.0 --print --print-hz 1.0

录制 MuJoCo 画面:
./build/g1_srbd_ik_walk_rp --duration 14.0 --print --print-hz 1.0 --record-video
ffmpeg -f rawvideo -pixel_format rgb24 -video_size 1200x800 -framerate 60 -i record/rgbRec.out -vf "vflip" -c:v libx264 -pix_fmt yuv420p record/g1_srbd_ik_walk_rp.mp4


./build/g1_srbd_ik_walk_rp_enhanced --duration 13.0 --print --print-hz 1

## ==== cd-nmpc+IK站立 ====

./build/g1_cd_nmpc_ik_stand --duration 13.0 --print --print-hz 1

## ==== cd-nmpc+ALIP+WBC两步行走 ====

./build/g1_cd_nmpc_wbc_walk --duration 13.0 --print --print-hz 1

./build/g1_cd_nmpc_ik_stand --duration 13.0 --print --print-hz 1

./g1_kino_nmpc_id_stand --duration 13.0 --print --print-hz 1.0