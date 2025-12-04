


(bam10) jdx@jdx:/media/jdx/jdx/bam$ python bam/hls/record.py --mass 0.4 --length 0.048 --logdir data_raw --trajectory sin_time_square --motor hls --kp 30 --damping 20


(bam10) jdx@jdx:/media/jdx/jdx/bam$ python bam/process.py --raw data_raw --logdir data_processed --dt 0.005


(bam10) jdx@jdx:/media/jdx/jdx/bam$ python bam/fit.py --actuator hls --model m1 --logdir data_processed --method cmaes --output params/hls/m1.json --trials 1000

(bam10) jdx@jdx:/media/jdx/jdx/bam$ python bam/plot.py --actuator hls --logdir data_processed --sim --params params/hls/m1.json
