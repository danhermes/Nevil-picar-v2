# kill everything in ros2 node list
ros2_cli="$(which ros2)"
"$ros2_cli" node list | while read N; do
  NAME="${N##*/}"
  echo "Stopping $N"
  pkill -SIGINT -f "__node:=$NAME" || pkill -TERM -f "__node:=$NAME" || pkill -9 -f "__node:=$NAME"
done