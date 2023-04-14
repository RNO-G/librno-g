#test environment script, not for production
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/build
export PATH=$PATH:`pwd`/build/test
export PYTHONPATH=$PYTHONPATH:`pwd`/build
