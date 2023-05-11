#test environment script, not for production
DIR_PATH="$(dirname "${BASH_SOURCE[0]}")"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${DIR_PATH}/build
export PATH=$PATH:${DIR_PATH}/build/test
export PYTHONPATH=$PYTHONPATH:${DIR_PATH}/build
