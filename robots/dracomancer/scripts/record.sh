#!/usr/bin/env bash
set -euo pipefail

bag_dir=$1
bag_name=$2
shift 2
if [[ -z "$bag_name" || "$bag_name" == */* ]]; then
  printf 'bag_name must be a nonempty filename component\n' >&2
  exit 2
fi

mkdir -p -- "$bag_dir"
stem="${bag_dir%/}/$(date +%Y%m%d_%H%M%S)_$bag_name"
bag="$stem.bag"

# Reserve rosbag's active output atomically. A second test of the final path
# covers another recorder completing its rename while we reserve the name.
for ((index = 2; ; index++)); do
  if [[ ! -e "$bag" && ! -L "$bag" && ! -e "$bag.active" && ! -L "$bag.active" ]] &&
     (set -o noclobber; : > "$bag.active") 2>/dev/null; then
    if [[ ! -e "$bag" && ! -L "$bag" ]]; then
      break
    fi
    rm -- "$bag.active"
  elif [[ ! -e "$bag" && ! -L "$bag" && ! -e "$bag.active" && ! -L "$bag.active" ]]; then
    printf 'Could not reserve rosbag output %s\n' "$bag.active" >&2
    exit 1
  fi
  bag="${stem}_$index.bag"
done

# Replace this process so roslaunch can stop the recorder directly.
exec rosrun rosbag record --all --output-name "$bag" "$@"
