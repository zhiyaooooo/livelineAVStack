start a server:
cd VehicleSim
git pull
julia --project --threads=auto
using VehicleSim
server(; no_mesh=true)

start a client:
Clone this repository into the same directory as VehicleSim
cd livelineAVStack
git pull
julia --project --threads=auto
# press ] to enter pkg mode
pkg> up VehicleSim
# press backspace to quit pkg mode
using Revise, Sockets, livelineAVStack
livelineAVStack.keyboard_client(ip"192.168.56.1")
