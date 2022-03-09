#!/bin/bash

# ========== Functions ==========
function run_all_combinations() {
  for map_resolution in "${map_resolutions[@]}"; do
    # Get the git commit ID (included in the log dir name for future reference)
    git_dir=$(rospack find wavemap_2d)/../.git
    git_commit_id=$(git --git-dir="${git_dir}" rev-parse --short --verify HEAD)

    for carmen_log_file_name in "${carmen_log_file_names[@]}"; do
      # Ensure that at most max_num_jobs jobs run in parallel
      while ((${num_jobs@P} >= max_num_jobs)); do
        wait -n
      done

      # Create the log dir and set the log file path
      run_log_dir="${output_log_dir}/${experiment_date}"
      mkdir -p "${run_log_dir}"
      run_log_file_prefix="${run_log_dir}/${carmen_log_file_name}_commit_${git_commit_id}_res_${map_resolution}_"

      # Run the experiments
      carmen_log_file_path="${carmen_log_dir}/${carmen_log_file_name}"
      "${executable}" -alsologtostderr -map_resolution "${map_resolution}" -carmen-log-file-path "${carmen_log_file_path}" -output_log_dir "${run_log_file_prefix}" &
    done
  done
}

# ========== Settings ==========
home_dir="/home/victor"
catkin_ws_dir="${home_dir}/catkin_ws"
carmen_log_dir="${home_dir}/data/2d_carmen_datasets"
output_log_dir="${home_dir}/data/2d_carmen_datasets/output"
package_name="wavemap_2d"
executable="${catkin_ws_dir}/devel/lib/${package_name}/carmen_processor"

declare -a carmen_log_file_names=(
  "fr079-complete.gfs"
)
# Options: ("fr079-complete.gfs"
         #  "aces_publicb.gfs"
         #  "belgioioso.gfs"
         #  "csail.corrected"
         #  "edmonton.gfs"
         #  "fr101.carmen.gfs"
         #  "fr-campus-20040714.carmen.gfs"
         #  "intel.gfs"
         #  "mexico.gfs"
         #  "MIT_Infinite_Corridor_2002_09_11_same_floor.gfs"
         #  "orebro.gfs"
         #  "seattle-r.gfs")

# ============ Vars =============
experiment_date=$(date '+%Y-%m-%d-%H-%M-%S')
num_jobs="\j" # The prompt escape to get the number of currently running jobs

# ============ Run =============
# Make sure the code is compiled
pushd "${catkin_ws_dir}" || exit
catkin build "${package_name}"
popd || exit

# Run the batches
declare -a map_resolutions=(0.1)
# Options: (0.20 0.1 0.05 0.02 0.01 0.005 0.002 0.001)
max_num_jobs=8
run_all_combinations
