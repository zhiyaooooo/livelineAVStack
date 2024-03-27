Clone this repository into the same directory as VehicleSim

issue: 
cd livelineAVStack
julia --project --threads=auto
using VehicleSim, Sockets, livelineAVStack
ivelineAVStack.keyboard_client(ip"192.168.56.1")
