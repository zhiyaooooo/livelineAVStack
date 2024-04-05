Start a server:  
cd VehicleSim  
git pull  
julia --project --threads=auto  
using VehicleSim  
#choose message types which you want the server to send to the client  
server(; no_mesh=true, measure_gt=true, measure_imu=true, measure_cam=true, measure_gps=true)  
#server(3; no_mesh=true, measure_gt=true, measure_cam=true)  

Start a client:  
Clone this repository into the same directory as VehicleSim  
cd livelineAVStack  
git pull  
julia --project --threads=auto  
#press ] to enter pkg mode  
pkg> up VehicleSim  # if new fixes have been pushed to VehicleSim, restart Julia
#press backspace to quit pkg mode  
using Revise, Sockets, livelineAVStack  
livelineAVStack.keyboard_client(ip"")  #enter your ip
