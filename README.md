Start a server:  
cd VehicleSim  
git pull  
julia --project --threads=auto  
using VehicleSim  
server(; no_mesh=true)  

Start a client:  
Clone this repository into the same directory as VehicleSim  
cd livelineAVStack  
git pull  
julia --project --threads=auto  
#press ] to enter pkg mode  
pkg> up VehicleSim  # if new fixes have been pushed to VehicleSim, restart Julia
#press backspace to quit pkg mode  
using Revise, Sockets, livelineAVStack  
livelineAVStack.keyboard_client(ip"172.20.10.2")  
