#!/bin/bash

set -o pipefail

# ======== Settings ========
home_dir="/home/victor"
dataset_dir="${home_dir}/data/2d_carmen_datasets"

url_common_prefixes="http://www2.informatik.uni-freiburg.de/~stachnis/datasets/datasets/"
declare -a url_unique_suffixes=(
  "csail/csail.corrected.log.gz"
  "freiburg-campus/fr-campus-20040714.carmen.gfs.log.gz"
  "intel-lab/intel.gfs.log.gz"
  "seattle-4-floor/seattle-r.gfs.log.gz"
  "MIT/MIT_Infinite_Corridor_2002_09_11_same_floor.gfs.log.gz"
  "orebro/orebro.gfs.log.gz"
  "belgioioso/belgioioso.gfs.log.gz"
  "aces/aces_publicb.gfs.log.gz"
  "edmonton/edmonton.gfs.log.gz"
  "fr079/fr079-complete.gfs.log.gz"
  "fr101/fr101.carmen.gfs.log.gz"
  "mexico/mexico.gfs.log.gz"
)

# ======== Download and extract ========
mkdir -p "${dataset_dir}"
pushd "${dataset_dir}" || exit
for url_unique_suffix in "${url_unique_suffixes[@]}"; do
  url="${url_common_prefixes}${url_unique_suffix}"
  name=$(cut -d "/" -f2 <<<"${url_unique_suffix}")
  name=${name%".log.gz"}
  wget -cq --show-progress -O - "${url}" | gunzip -c >"${name}"
done
popd || exit
