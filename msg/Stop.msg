# some constants
int8 STOP_CURRENT = 1   # stop immediately and go to next goal in task
int8 STOP_ALL = 2       # stop immediately and end entire task
int8 END_TASK = 3       # end task after this goal. Will change if the two above it are set before task finishes.
int8 CONTINUE = 4
int8 CANCEL = 5
# main message variable
int8 stop