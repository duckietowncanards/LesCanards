source /environment.sh
dt-launchfile-init

# locate the web app via ROS. duckietown_web is a plain (non-ROS) package that sits
# beside path_following under packages/, so resolve it as a sibling of that package.
WEB_DIR="$(rospack find path_following)/../duckietown_web"

dt-exec roslaunch path_following path_nodes.launch veh:=$VEHICLE_NAME
dt-exec uvicorn main:app --host 0.0.0.0 --port 8001 --app-dir "$WEB_DIR"

dt-launchfile-join