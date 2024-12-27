#test environment script, not for production
DIR_PATH="$(dirname "${BASH_SOURCE[0]}")"

export LD_LIBRARY_PATH=${DIR_PATH}/build:$LD_LIBRARY_PATH
export PATH=$PATH:${DIR_PATH}/build/test
export PYTHONPATH=$PYTHONPATH:${DIR_PATH}/build
