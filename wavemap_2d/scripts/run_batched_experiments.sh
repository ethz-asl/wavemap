#!/bin/bash

# ========== Settings ==========
catkin_ws_dir="/home/victor/catkin_ws"
carmen_log_dir="/mnt/scratch/data/2d_datasets"
output_log_dir="/mnt/scratch/data/2d_datasets/output"

executable="${catkin_ws_dir}/devel/lib/wavemap_2d/wavemap_carmen_processor"
max_num_jobs=14

declare -a carmen_log_file_names=(
  "aces_publicb.gfs.log"
  "fhw-rec-001.gfs.log"
  "fr-campus-20040714.carmen.gfs.log"
  "MIT_Infinite_Corridor_2002_09_11_same_floor.gfs.log"
  "belgioioso.gfs.log"
  "fr079-complete.gfs.log"
  "intel.gfs.log"
  "orebro.gfs.log"
  "edmonton.gfs.log"
  "fr101.carmen.gfs.log"
  "mexico.gfs.log"
  "seattle-r.gfs.log"
)

declare -a map_resolutions=(0.20)  # 0.1 0.05 0.02 0.01)

# ============ Run =============
experiment_date=$(date '+%Y-%m-%d-%H-%M-%S')

num_jobs="\j" # The prompt escape to get the number of currently running jobs
for map_resolution in "${map_resolutions[@]}"; do
  git_dir=$(rospack find wavemap_2d)/../.git
  git_commit_id=$(git --git-dir=${git_dir} rev-parse --verify HEAD)

  for carmen_log_file_name in "${carmen_log_file_names[@]}"; do
    # Ensure that at most max_num_jobs jobs run in parallel
    while ((${num_jobs@P} >= max_num_jobs)); do
      wait -n
    done

    carmen_log_file_path="${carmen_log_dir}/${carmen_log_file_name}"
    run_log_dir="${output_log_dir}/${carmen_log_file_name}"
    mkdir -p "${run_log_dir}"
    run_log_file_prefix="${run_log_dir}/${experiment_date}_commit_${git_commit_id}_res_${map_resolution}_"
    "${executable}" -alsologtostderr -map_resolution ${map_resolution} -carmen-log-file-path "${carmen_log_file_path}" -output_log_dir "${run_log_file_prefix}" &
  done
done
