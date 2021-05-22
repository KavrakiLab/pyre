#/bin/bash
cd "${0%/*}"

dataset=$1
exp=$2

if [ -z "$exp" ]
then
    echo "Usage: [dataset] [experiment]"
    exit 0
fi

find ./$dataset/*.log -type f -exec sed -i 's/Experiment .*$/Experiment '"$exp"'/g' {} \;

./ompl_benchmark_statistics.py ./$dataset/*.log -d ./$dataset\_$exp.db
