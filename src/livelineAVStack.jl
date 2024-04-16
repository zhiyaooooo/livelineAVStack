module livelineAVStack

using VehicleSim
using LinearAlgebra
using SparseArrays
using Sockets
using Serialization
using StaticArrays
using Ipopt
using Symbolics 

include("client.jl")
include("example_project.jl")
include("trajectory_functions.jl")

end # module livelineAVStack
